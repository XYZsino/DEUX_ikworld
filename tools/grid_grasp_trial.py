"""ROS 2 Jazzy grid pose linkage. Default: preview only, zero command publishers.

Explicit enable_execution:=true AND /task/grid_grasp action request are needed
for motion. Never moves to ready automatically; the full ready snapshot is checked.
"""
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_names_and_types
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from deux_task_msgs.msg import ObjectDetectionArray
from deux_task_msgs.action import MoveToReady
from deux_task_core.model import ARM_NAMES
from deux_task_runtime.common import RuntimeNode, ExclusiveAction, seconds, pose_matrix, joint_array, ros_time, pause, cancel_children
from grid_grasp_runtime import GridLibrary, ROOT, XY_TOLERANCE_M
from parallel_grid_hand import close_during_descent


class GridTrial(RuntimeNode):
    library_class = GridLibrary
    def __init__(self):
        super().__init__('grid_grasp_trial', observe_status=True,
                         parameter_overrides=[Parameter('config_file', value=str(ROOT/'src/deux_task_bringup/config/installation.yaml'))])
        self.declare_parameter('enable_execution', False)
        self.declare_parameter('stationary_left_file', '')
        self.declare_parameter('recommended_only', False)
        self.declare_parameter('target_wait_timeout_s', 120.)
        self.declare_parameter('grid_library_directory', '')
        self.declare_parameter('use_observed_yaw', False)
        self.declare_parameter('grid_output_directory', str(ROOT/'.setup/grid-grasp-test'))
        self.enabled = self.get_parameter('enable_execution').value
        self.library = self.library_class(self.config, recommended=self.get_parameter('recommended_only').value,
            stationary_left_file=self.get_parameter('stationary_left_file').value or None,
            directory=self.get_parameter('grid_library_directory').value or None,
            yaw_secondary=self.get_parameter('use_observed_yaw').value,
            load_rotation_paths=not self.get_parameter('use_observed_yaw').value); self.robot = self.library.robot
        # Match the existing runtime contract: /deux/estop is an event input,
        # not a heartbeat/state service. Never publish a synthetic clear event.
        self.predictions = None; self.arm = None; self.arm_received = 0.; self.estop = False; self.estop_seen = False
        self.calibration = {}; self.cal_at = 0.; self.track = {}; self.track_at = 0.
        self.samples = []; self.last_sample = None; self.external = -float('inf')
        self.out = Path(self.get_parameter('grid_output_directory').value); self.out.mkdir(parents=True, exist_ok=True)
        self.preview = dict(selectable=False, reason='waiting for observations'); self.used = False
        self.locked_selection = None
        self.execution_warnings = []
        self.max_close_path_error_m = 0.
        self.motion_samples = []; self.last_motion_log_at = -float('inf')
        self.create_subscription(ObjectDetectionArray, '/task/prediction_detections', self.on_prediction, qos_profile_sensor_data)
        self.create_subscription(JointState, '/arm/joint_states', self.on_arm, qos_profile_sensor_data)
        self.create_subscription(Bool, '/deux/estop', self.on_estop, 10)
        self.create_subscription(String, '/task/camera_calibration_status', self.on_calibration, 10)
        self.create_subscription(String, '/task/icp_preview_status', self.on_track, 10)
        self.create_subscription(JointTrajectory, '/arm/deux_arm_controller/joint_trajectory', lambda m: setattr(self, 'external', time.monotonic()), 10)
        self.publisher = self.create_publisher(String, '/task/grid_grasp/preview', 10)
        self.create_timer(.1, self.tick, callback_group=self.group)
        self.client = None; self.hand = None
        if self.enabled:
            from right_trial_hand import RightTrialHand
            self.client = ActionClient(self, FollowJointTrajectory, '/arm/deux_arm_controller/follow_joint_trajectory', callback_group=self.group)
            self.hand = RightTrialHand(self)
        self.action = ExclusiveAction(self, MoveToReady, '/task/grid_grasp', self.execute, self.admit)

    def on_arm(self, m): self.arm = m; self.arm_received = time.monotonic()
    def on_estop(self, m): self.estop = m.data; self.estop_seen = True
    def on_calibration(self, m): self.calibration = json.loads(m.data); self.cal_at = time.monotonic()
    def on_track(self, m): self.track = json.loads(m.data); self.track_at = time.monotonic()

    def on_prediction(self, m):
        if self.predictions is not None and seconds(m.header.stamp) <= seconds(self.predictions.header.stamp): return
        self.predictions = m
        identity = self.track.get('target_id'); stamp = seconds(m.header.stamp)
        found = [d for d in m.detections if d.track_id == identity]
        if len(found) != 1: self.samples = []; return
        if self.last_sample is None or self.last_sample[0] != identity or stamp-self.last_sample[1] > .5: self.samples = []
        self.samples.append(stamp); self.samples = self.samples[-20:]; self.last_sample = identity, stamp

    def tracked_observation(self):
        if not self.calibration.get('valid') or time.monotonic()-self.cal_at > 2.: raise ValueError('camera calibration unavailable')
        if not self.track.get('tracking') or time.monotonic()-self.track_at > 1.: raise ValueError('ROI target unavailable')
        m = self.predictions
        if m is None: raise ValueError('prediction topic unavailable')
        if m.header.frame_id != 'base_link' or m.model_id != self.config['model_id']: raise ValueError('prediction frame/model mismatch')
        identity = self.track.get('target_id'); found = [d for d in m.detections if d.track_id == identity]
        if len(found) != 1 or len(self.samples) < 3 or self.samples[-1]-self.samples[0] < .2: raise ValueError('stable same-ID XY samples required')
        d = found[0]; v = d.velocity
        return identity, pose_matrix(d.pose.pose), np.array([v.linear.x, v.linear.y, v.linear.z, v.angular.z]), seconds(m.header.stamp)

    def measured(self):
        if (self.arm is None or time.monotonic()-self.arm_received > self.config['max_joint_age']
                or not 0 <= self.now()-seconds(self.arm.header.stamp) <= self.config['max_joint_age']):
            raise ValueError('arm feedback stale')
        return joint_array(self.arm, ARM_NAMES)

    def ready_error(self):
        expected = np.r_[self.library.ready[:7], self.library.ready[14:21]]
        return float(np.max(abs(self.measured()-expected)))

    def gate(self, goal=None):
        if goal is not None and goal.is_cancel_requested: raise InterruptedError('operator canceled')
        if self.estop: raise ValueError('e-stop active')
        self.hardware_gate(); self.check_model_version()
        if time.monotonic()-self.external < 1.:
            raise ValueError('competing direct arm trajectory received')
        actual = self.measured()
        if np.max(abs(actual[:7]-self.library.ready[:7])) > .03: raise ValueError('stationary left arm differs from validated ready snapshot')
        return actual

    def tick(self):
        data = dict(preview_only=not self.enabled, execution_enabled=self.enabled, motion_requested=self.action.busy,
                    cells=len(self.library.entries), selectable=False, arm_ready=False, timestamp_s=self.now(),
                    selection_policy='observed XY only; user-fixed Z=0.97m and -Y speed=0.09m/s; fixed orientation assumption')
        if self.library.yaw_secondary:
            data.update(selection_policy='XY and arrival first; observed yaw ranks rotations only at the chosen cell',
                        contact_cells=len(self.library.cell_options), rotation_preference='yaw only; no orientation rejection gate')
        try: data.update(ready_error_rad=self.ready_error(), arm_ready=self.ready_error() <= .03)
        except Exception as error: data['arm_reason'] = str(error)
        try:
            identity, obj, velocity, stamp = self.tracked_observation()
            live_grasp, _ = self.library.grasp_now(obj, velocity, stamp, self.now())
            data.update(target_id=identity, observed_object_xyz_m=obj[:3, 3].tolist(),
                        observed_grasp_xyz_m=live_grasp[:3, 3].tolist(), observed_velocity_m_s=velocity[:3].tolist(),
                        grasp_z_source='user-fixed physical grasp plane base_link Z=0.97m; estimator Z is model center')
            data.update(self.library.select(obj, velocity, stamp, self.now()), target_id=identity, selectable=True)
            data['reason'] = 'candidate available' if data['arm_ready'] else 'candidate available; actual arms are not at the validated ready snapshot'
        except Exception as error: data['reason'] = str(error)
        data['dispatch_ready'] = data['selectable'] and data['arm_ready']
        data['replay_policy'] = 'freeze cell/timing at dispatch; perception changes do not cancel playback'
        data['diagnostic_policy'] = 'after dispatch: monitoring errors log only; operator cancel and e-stop enforced'
        if self.action.busy and self.locked_selection is not None:
            data.update(locked_cell_id=self.locked_selection['cell_id'],
                        trajectory_start_at_s=self.locked_selection['start_at_s'],
                        vision_reason=data['reason'], reason='stored replay committed; vision is diagnostic only')
        if self.enabled and not self.action.busy:
            try:
                self.admit(None); data.update(execution_admission_ready=True, execution_reason=getattr(self, 'admission_note', 'hardware, hand profile and ready snapshot admitted'))
            except Exception as error:
                data.update(execution_admission_ready=False, execution_reason=str(error))
        if hasattr(self, 'phase_details'): data.update(self.phase_details())
        self.preview = data
        self.publisher.publish(String(data=json.dumps(data)))
        temporary = self.out/'preview.tmp'; temporary.write_text(json.dumps(data, indent=2)); temporary.replace(self.out/'preview.json')

    def admit(self, request):
        if not self.enabled: raise ValueError('preview-only mode; no arm/hand commands are enabled')
        if self.used: raise ValueError('one-shot trial already used; restart required')
        self.gate()
        if self.ready_error() > .03: raise ValueError('actual arms must already match validated ready snapshot')
        if not self.client.server_is_ready(): raise ValueError('arm trajectory action unavailable')
        competing = {'/task/move_to_ready', '/task/run', '/task/execute_motion'}
        if any(name in competing for name, types in get_action_names_and_types(self)):
            raise ValueError('another task/ready execution action is registered')
        self.hand.guard()

    def warn_execution(self, reason):
        if reason not in self.execution_warnings:
            self.execution_warnings.append(reason)
            self.get_logger().warning(reason)

    def committed_gate(self, goal):
        # Operator/e-stop are intentional stops; monitoring errors are diagnostic.
        if goal is not None and goal.is_cancel_requested: raise InterruptedError('operator canceled')
        if self.estop or (self.hand is not None and getattr(self.hand, 'estop', False)):
            raise InterruptedError('e-stop active')
        try: return self.gate(goal)
        except Exception as error:
            self.warn_execution(str(error))
            try: return joint_array(self.arm, ARM_NAMES)
            except Exception: return np.full(14, np.nan)

    @staticmethod
    def replay_goal(trajectory):
        # ROS FollowJointTrajectory: -1 disables these controller error tolerances.
        # Controller/driver faults can still stop motion independently of this node.
        tolerances = [JointTolerance(name=name, position=-1., velocity=-1., acceleration=-1.)
                      for name in trajectory.joint_names]
        return FollowJointTrajectory.Goal(trajectory=trajectory,
                                          path_tolerance=tolerances, goal_tolerance=tolerances)

    def validate_replay(self, goal, curve, start_at, close=False):
        """Hardware/path monitoring only. Never reads object or camera estimates."""
        actual = self.committed_gate(goal) if self.locked_selection is not None else self.gate(goal)
        elapsed = self.now()-start_at
        if elapsed >= 0 and np.max(abs(actual[7:]-curve(min(elapsed, 5.)))) > self.config['tracking_tolerance']:
            self.warn_execution('stored arm trajectory tracking error')
        if close:
            q = self.library.ready.copy(); q[14:21] = actual[7:]
            feedback_at = seconds(self.arm.header.stamp)
            expected_q = self.library.ready.copy(); expected_q[14:21] = curve(np.clip(feedback_at-start_at, 0, 5.))
            actual_pose = self.robot.ee_fk('right', q)
            error = float(np.linalg.norm(actual_pose[:3, 3]-self.robot.ee_fk('right', expected_q)[:3, 3]))
            # Grasp alignment is diagnostic during committed playback. The
            # configured joint tracking/driver/stop guards above remain active.
            self.max_close_path_error_m = max(self.max_close_path_error_m, error)
        return actual

    def validate_dispatch(self, selected, identity, goal, curve, start_at):
        """The final perception gate is BEFORE handing the path to the controller."""
        actual = self.validate_replay(goal, curve, start_at)
        tid, obj, velocity, stamp = self.tracked_observation()
        if tid != identity: raise ValueError('target ID changed before dispatch')
        self.library.live_error(selected, obj, velocity, stamp, self.now())
        return actual

    def log_replay_sample(self, curve, start_at, actual):
        """20Hz memory log. Diagnostic failures never affect committed replay."""
        now = self.now()
        if now-self.last_motion_log_at < .05: return
        self.last_motion_log_at = now
        row = dict(time_s=now, elapsed_s=now-start_at)
        try:
            expected = np.asarray(curve(np.clip(now-start_at, 0., float(curve.x[-1]))))
            q = self.library.ready.copy(); q[14:21] = actual[7:]
            wanted = self.library.ready.copy(); wanted[14:21] = expected
            actual_pose = self.robot.ee_fk('right', q); planned_pose = self.robot.ee_fk('right', wanted)
            row.update(actual_right_rad=actual[7:].tolist(), planned_right_rad=expected.tolist(),
                       max_joint_error_rad=float(np.max(abs(actual[7:]-expected))),
                       actual_grasp_xyz_m=actual_pose[:3, 3].tolist(), planned_grasp_xyz_m=planned_pose[:3, 3].tolist(),
                       cartesian_path_error_m=float(np.linalg.norm(actual_pose[:3, 3]-planned_pose[:3, 3])))
            sample = self.hand.raw.get('right') if self.hand is not None and hasattr(self.hand, 'raw') else None
            if sample is not None: row['right_hand_feedback_rad'] = dict(sample[1])
            try:
                identity, obj, velocity, stamp = self.tracked_observation()
                grasp, _ = self.library.grasp_now(obj, velocity, stamp, now)
                predicted = np.array(self.locked_selection['contact_xyz_m'])+self.library.velocity*(now-self.locked_selection['contact_at_s'])
                row.update(observed_target_id=identity, observed_grasp_xy_m=grasp[:2, 3].tolist(),
                           observed_model_z_m=float(obj[2, 3]), observation_age_s=now-stamp,
                           prediction_xy_error_m=float(np.linalg.norm(grasp[:2, 3]-predicted[:2])), vision_available=True)
            except Exception as error: row.update(vision_available=False, vision_reason=str(error))
        except Exception as error: row['diagnostic_error'] = str(error)
        self.motion_samples.append(row)

    async def execute(self, goal):
        result = MoveToReady.Result(); handle = None; pending = None; hand_task = None; report = {}; stop_requested = False
        try:
            self.max_close_path_error_m = 0.
            self.motion_samples = []; self.last_motion_log_at = -float('inf')
            self.gate(goal)
            await self.hand.open_ready(goal)
            deadline = time.monotonic()+float(self.get_parameter('target_wait_timeout_s').value)
            while True:
                self.gate(goal)
                if self.ready_error() > .03: raise ValueError('ready pose changed before dispatch')
                try:
                    identity, obj, velocity, stamp = self.tracked_observation()
                    selected = self.library.select(obj, velocity, stamp, self.now()); break
                except ValueError:
                    if time.monotonic() > deadline: raise TimeoutError('no eligible grid encounter before target wait timeout')
                    await pause(self, .02)
            self.used = True; key = selected['cell_id']; p = self.library.entries[key]; curve = self.library.curves[key]
            start_at = selected['start_at_s']; report.update(selected, target_id=identity)
            report['replay_policy'] = 'frozen after dispatch; no perception-triggered cancellation'
            tr = JointTrajectory(joint_names=p['joint_names']); tr.header.stamp = ros_time(start_at)
            for t, q, v in zip(p['times_s'], p['positions_rad'], p['velocities_rad_s']):
                tr.points.append(JointTrajectoryPoint(positions=q, velocities=v, time_from_start=ros_time(t, Duration)))
            self.validate_dispatch(selected, identity, goal, curve, start_at)
            self.locked_selection = dict(selected)
            pending = self.client.send_goal_async(self.replay_goal(tr))
            while not pending.done():
                actual = self.validate_replay(goal, curve, start_at)
                self.log_replay_sample(curve, start_at, actual)
                if self.now() >= start_at-.1: self.warn_execution('late arm action acceptance')
                await pause(self, .005)
            handle = pending.result()
            if not handle.accepted: raise ValueError('arm controller rejected stored trajectory')
            if self.now() >= start_at-.1: self.warn_execution('accepted trajectory too late')
            arm_result = handle.get_result_async()
            def guard(): self.committed_gate(goal)
            def qualified(elapsed):
                self.validate_replay(goal, curve, start_at, close=True); return True
            hand_task = self.executor.create_task(close_during_descent, p, start_at, self.hand,
                now=self.now, pause=lambda dt: pause(self, dt), guard=guard, qualified=qualified, strict_timing=False)
            watchdog = time.monotonic()+max(0., start_at-self.now())+8.
            while not arm_result.done() or not hand_task.done():
                elapsed = self.now()-start_at
                actual = self.validate_replay(goal, curve, start_at)
                self.log_replay_sample(curve, start_at, actual)
                if elapsed > 3.:
                    try: self.hand.guard()
                    except Exception as error: self.warn_execution(str(error))
                if hand_task.done():
                    if hand_task.exception() is not None: raise hand_task.exception()
                elif elapsed > 3.04: report['parallel_closure_deadline_warning'] = True
                if time.monotonic() > watchdog: self.warn_execution('stored motion completion overdue; waiting without cancel')
                await pause(self, .005)
            response = arm_result.result()
            if response.status != 4 or response.result.error_code: raise ValueError('arm controller failed: '+response.result.error_string)
            if not hand_task.done() or hand_task.exception(): raise ValueError('parallel hand scheduling failed')
            report['hand_timing'] = hand_task.result()
            if np.max(abs(self.committed_gate(goal)[7:]-curve(5.))) > self.config['final_tolerance']: self.warn_execution('final joint error')
            handle = None; result.success = True
            result.reason = 'Stored arm path and parallel hand commands completed; physical grasp success unverified'
        except Exception as error:
            stop_requested = isinstance(error, InterruptedError) or self.estop or goal.is_cancel_requested
            result.success = False; result.reason = f'{type(error).__name__}: {error}'
        finally:
            if hand_task is not None and not hand_task.done(): hand_task.cancel()
            if stop_requested and pending is not None and handle is None:
                def late(future):
                    try:
                        h = future.result()
                        if h.accepted: h.cancel_goal_async()
                    except Exception: pass
                pending.add_done_callback(late)
            if stop_requested: await cancel_children(self, [handle])
            report.update(success=result.success, reason=result.reason, physical_grasp_verified=False,
                          max_close_path_error_m=self.max_close_path_error_m, execution_warnings=self.execution_warnings)
            if self.motion_samples:
                filename = 'motion-trace-'+str(round(self.motion_samples[0]['time_s']*1e9))+'.jsonl'
                try:
                    (self.out/filename).write_text(''.join(json.dumps(row)+'\n' for row in self.motion_samples))
                    report.update(motion_trace=filename, motion_samples=len(self.motion_samples))
                except OSError as error: report['motion_trace_error'] = str(error)
            (self.out/'result.json').write_text(json.dumps(report, indent=2))
            self.locked_selection = None
        return result


def main():
    rclpy.init(); node = GridTrial()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
