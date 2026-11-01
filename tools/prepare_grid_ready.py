"""Fresh-current -> saved RIGHT ready planning and optional explicitly requested execution.

Left arm stays measured and stationary. No hand commands. Plan-only by default.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from builtin_interfaces.msg import Duration
from deux_task_core.model import RobotModel, ARM_NAMES
from deux_task_core.prepare_route import knots, validate_path
from deux_task_runtime.common import RuntimeNode, joint_array, current, ros_time
from grid_grasp_runtime import GridLibrary, ROOT, sha
from upper_collision_batch import checked_clearances


class Preparation(RuntimeNode):
    def __init__(self):
        super().__init__('grid_right_ready_preparation', observe_status=True,
                         parameter_overrides=[Parameter('config_file', value=str(ROOT/'src/deux_task_bringup/config/installation.yaml'))])
        self.robot = RobotModel(self.config['robot']); self.arm = None; self.received = 0.; self.estop = False; self.external = -float('inf')
        self.create_subscription(JointState, '/arm/joint_states', self.on_arm, qos_profile_sensor_data)
        self.create_subscription(Bool, '/deux/estop', lambda m: setattr(self, 'estop', m.data), 10)
        self.create_subscription(JointTrajectory, '/arm/deux_arm_controller/joint_trajectory', lambda m: setattr(self, 'external', time.monotonic()), 10)
        self.client = ActionClient(self, FollowJointTrajectory, '/arm/deux_arm_controller/follow_joint_trajectory')
        self.out = ROOT/'.setup/grid-grasp-test'; self.out.mkdir(parents=True, exist_ok=True)
    def on_arm(self, msg): self.arm = msg; self.received = time.monotonic()
    def actual(self):
        if self.arm is None or time.monotonic()-self.received > .2 or not current(self.arm.header.stamp, self.now(), .2):
            raise RuntimeError('fresh arm feedback required')
        return joint_array(self.arm, ARM_NAMES)
    def guard(self, left):
        if self.estop: raise InterruptedError('e-stop event received')
        self.hardware_gate(); self.check_model_version()
        if time.monotonic()-self.external < 1.: raise RuntimeError('competing direct arm trajectory received')
        actual = self.actual()
        if np.max(abs(actual[:7]-left)) > .03: raise RuntimeError('stationary left arm changed')
        return actual
    def wait(self, future, timeout, guard):
        deadline = float('inf') if timeout is None else time.monotonic()+timeout
        while not future.done():
            rclpy.spin_once(self, timeout_sec=.01); guard()
            if time.monotonic() > deadline: raise TimeoutError('controller response timeout')
        return future.result()


def main():
    parser = argparse.ArgumentParser(description=__doc__); parser.add_argument('--execute', action='store_true')
    parser.add_argument('--log-errors', action='store_true', help='After dispatch, record monitoring errors without canceling; e-stop remains active')
    args = parser.parse_args(); rclpy.init(); node = Preparation(); handle = None; pending = None
    report = dict(execution_requested=args.execute, hand_commands=False, left_arm_commanded=False, warnings=[])
    intentional_stop = False
    def monitor(left):
        if not args.log_errors: return node.guard(left)
        try: return node.guard(left)
        except InterruptedError: raise
        except Exception as error:
            message = str(error)
            if message not in report['warnings']:
                report['warnings'].append(message); node.get_logger().warning(message)
            return joint_array(node.arm, ARM_NAMES)
    def diagnostic(message):
        if not args.log_errors: raise RuntimeError(message)
        if message not in report['warnings']:
            report['warnings'].append(message); node.get_logger().warning(message)
    try:
        deadline = time.monotonic()+8
        while (node.arm is None or node.robot_status is None) and time.monotonic() < deadline: rclpy.spin_once(node, timeout_sec=.1)
        actual = node.actual(); q = np.array(node.config['home_joints']); q[:7] = actual[:7]; q[14:21] = actual[7:]
        node.guard(actual[:7]); goal = q.copy(); goal[14:21] = node.config['home_joints'][14:21]
        route = knots(node.config, q, goal, node.robot)
        times = [0.]; rows = [route[0]]
        for a, b in zip(route[:-1], route[1:]):
            delta = abs(b[14:21]-a[14:21])
            duration = max(1., float(np.max(1.5*delta/np.minimum(node.robot.velocity[14:21], .15))),
                           float(np.sqrt(np.max(6*delta/np.minimum(node.robot.acceleration[14:21], .3)))))+.2
            times.append(times[-1]+duration); rows.append(b)
        path = np.array(rows); curve = CubicHermiteSpline(times, path, np.zeros_like(path))
        dense = curve(np.linspace(0, times[-1], int(np.ceil(times[-1]/.01))+1))
        margin = float(np.min(checked_clearances(node.robot, dense, node.config['obstacles'], node.config['collision_margin'])))
        certified = validate_path(node.robot, path)
        # Revalidate every recommended pick path with the ACTUAL stationary left arm.
        library = GridLibrary(node.config); validated = {}; left = actual[:7].copy()
        for key, stored in library.entries.items():
            tt = np.linspace(0, 5., 501); states = np.tile(library.ready, (len(tt), 1))
            states[:, :7] = left; states[:, 14:21] = library.curves[key](tt)
            validated[key] = float(np.min(checked_clearances(node.robot, states, node.config['obstacles'], node.config['collision_margin'])))
        session = dict(left_arm_rad=left.tolist(), manifest_sha256=sha(ROOT/'artifacts/right_grasp_grid_fast/manifest.json'),
                       validated_cells=validated, created_at_s=node.now(), geometry='existing model and measured visible-surface screening')
        report.update(start_arm_rad=actual.tolist(), goal_right_rad=goal[14:21].tolist(), duration_s=times[-1],
                      right_waypoints=path[:, 14:21].tolist(), times_s=times, min_margin_m=margin,
                      continuous_conveyor_margin_m=certified, revalidated_pick_cells=len(validated))
        (node.out/'ready-plan.json').write_text(json.dumps(report, indent=2))
        (node.out/'stationary-left.json').write_text(json.dumps(session, indent=2))
        print(json.dumps(report, indent=2), flush=True)
        if not args.execute: return
        # Drain messages after CPU-bound planning and reject any changed start.
        for _ in range(20): rclpy.spin_once(node, timeout_sec=.005)
        if np.max(abs(node.guard(left)-actual)) > .03: raise RuntimeError('start changed during planning')
        if not node.client.wait_for_server(timeout_sec=3.): raise RuntimeError('arm action unavailable')
        trajectory = JointTrajectory(joint_names=list(ARM_NAMES[7:]))
        trajectory.header.frame_id = 'base_link'
        trajectory.header.stamp = ros_time(node.now()+.75)
        for t, row in zip(times, path): trajectory.points.append(JointTrajectoryPoint(positions=row[14:21].tolist(), velocities=[0.]*7, time_from_start=ros_time(t, Duration)))
        start_at = trajectory.header.stamp.sec+trajectory.header.stamp.nanosec/1e9
        action_goal = FollowJointTrajectory.Goal(trajectory=trajectory)
        if args.log_errors:
            tolerance = [JointTolerance(name=name, position=-1., velocity=-1., acceleration=-1.) for name in trajectory.joint_names]
            action_goal.path_tolerance = tolerance; action_goal.goal_tolerance = tolerance
        pending = node.client.send_goal_async(action_goal)
        handle = node.wait(pending, None if args.log_errors else .5, lambda: monitor(left))
        if not handle.accepted: raise RuntimeError('ready trajectory rejected')
        if node.now() >= start_at: diagnostic('late ready trajectory acceptance')
        future = handle.get_result_async(); deadline = time.monotonic()+times[-1]+5
        while not future.done():
            rclpy.spin_once(node, timeout_sec=.01); measured = monitor(left)
            elapsed = np.clip(node.now()-start_at, 0., times[-1])
            if np.max(abs(measured[7:]-curve(elapsed)[14:21])) > node.config['tracking_tolerance']: diagnostic('ready tracking error')
            if time.monotonic() > deadline: diagnostic('ready completion overdue')
        response = future.result()
        if response.status != 4 or response.result.error_code: raise RuntimeError('ready action failed: '+response.result.error_string)
        final = monitor(left); error = float(np.max(abs(final[7:]-goal[14:21])))
        if error > .03: diagnostic('ready final error >0.03rad')
        handle = None; report.update(success=True, final_max_error_rad=error, final_arm_rad=final.tolist())
        print(json.dumps(dict(success=True, final_max_error_rad=error)), flush=True)
    except (Exception, KeyboardInterrupt) as error:
        intentional_stop = isinstance(error, (InterruptedError, KeyboardInterrupt))
        report.update(success=False, reason=f'{type(error).__name__}: {error}'); print(json.dumps(report), flush=True)
        raise
    finally:
        if (not args.log_errors or intentional_stop) and handle is not None and handle.accepted:
            cancel = handle.cancel_goal_async()
            try: node.wait(cancel, 2., lambda: None)
            except Exception: pass
        elif (not args.log_errors or intentional_stop) and pending is not None and not report.get('success'):
            def late(future):
                try:
                    h = future.result()
                    if h.accepted: h.cancel_goal_async()
                except Exception: pass
            pending.add_done_callback(late)
            for _ in range(20): rclpy.spin_once(node, timeout_sec=.02)
        (node.out/'ready-result.json').write_text(json.dumps(report, indent=2))
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
