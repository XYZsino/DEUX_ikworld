"""ROI -> XAlign -> AwaitGrasp -> committed descent/close/lift. Preview by default."""
import json
import time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
from scipy.interpolate import CubicHermiteSpline
import rclpy
from rclpy.action.graph import get_action_names_and_types
from rclpy.executors import ExternalShutdownException
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.action import MoveToReady
from deux_task_runtime.common import pause, ros_time, cancel_children
from grid_grasp_trial import GridTrial
from roi_grasp_planner import RoiPlanner, encounter, load_policy, DEFAULT_POLICY
from parallel_grid_hand import close_during_descent


class ControllerFailure(RuntimeError):
    """Controller rejected/ended motion; never automatically retry that motion."""


class RoiTrial(GridTrial):
    def __init__(self):
        super().__init__()
        self.declare_parameter('roi_policy_file', str(DEFAULT_POLICY))
        self.declare_parameter('output_directory', str(DEFAULT_POLICY.parents[1]/'artifacts/roi-grasp-runs'))
        self.policy = load_policy(self.get_parameter('roi_policy_file').value)
        self.out = type(DEFAULT_POLICY)(self.get_parameter('output_directory').value)
        self.out.mkdir(parents=True, exist_ok=True)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.right_hand_ready_fraction = 0.  # Keep the hand fully open while aligning/waiting.
        self.phase = 'Idle'; self.phase_reason = 'explicit action request required'
        self.x_error = None; self.target_id = None; self.planned_selection = None

    def phase_details(self):
        selected = self.locked_selection or getattr(self, 'planned_selection', None)
        data = dict(workflow='ROI fixed-speed grasp', phase=getattr(self, 'phase', 'Idle'),
                    phase_reason=getattr(self, 'phase_reason', ''),
                    x_error_m=getattr(self, 'x_error', None),
                    selected_target_id=getattr(self, 'target_id', None),
                    dispatch_ready=False)
        if selected:
            data.update(planned_cell_id=selected['cell_id'], cell_id=selected['cell_id'],
                        target_id=selected.get('target_id', self.target_id), contact_xyz_m=selected['contact_xyz_m'],
                        contact_in_s=selected['contact_at_s']-self.now(), start_in_s=selected['start_at_s']-self.now(),
                        selection_frozen=True, replay_policy='cell and contact time frozen at selection; perception is diagnostic only')
        return data

    def admit(self, request):
        if not self.enabled: raise ValueError('preview-only mode')
        if self.used: raise ValueError('one-shot trial already used; restart required')
        self.stop_guard(None)
        self.admission_note = 'request accepted; hardware and hand inputs currently available'
        try: self.full_state(None); self.hand.guard()
        except ValueError as error:
            self.admission_note = 'request can wait for readiness: '+str(error)
            self.warn_execution(self.admission_note)
        if any(name in {'/task/move_to_ready', '/task/run', '/task/execute_motion'}
               for name, _ in get_action_names_and_types(self)):
            raise ValueError('another motion action is registered')

    def full_state(self, goal):
        actual = self.gate(goal)
        if len(self.arm.name) != len(self.arm.velocity):
            self.warn_execution('joint velocity feedback missing; velocity is diagnostic only')
        velocity = dict(zip(self.arm.name, self.arm.velocity))
        speeds = np.asarray([velocity.get(f'arm_r_joint{i}', np.nan) for i in range(1, 8)])
        if not np.isfinite(speeds).all() or np.max(abs(speeds)) > .03:
            self.warn_execution('arm velocity exceeds 0.03rad/s or is unavailable; diagnostic only')
        q = self.library.ready.copy(); q[:7] = actual[:7]; q[14:21] = actual[7:]
        return q

    async def plan(self, goal, function, *args):
        future = self.pool.submit(function, *args)
        while not future.done():
            self.stop_guard(goal)
            await pause(self, .02)
        return future.result()

    async def play(self, goal, plan, start_at, selected=None):
        """Do not replace or cancel a dispatched segment for measured errors."""
        curve = CubicHermiteSpline(plan['times_s'], plan['positions_rad'], plan['velocities_rad_s'])
        tr = JointTrajectory(joint_names=plan['joint_names']); tr.header.stamp = ros_time(start_at)
        for t, q, v in zip(plan['times_s'], plan['positions_rad'], plan['velocities_rad_s']):
            tr.points.append(JointTrajectoryPoint(positions=q, velocities=v, time_from_start=ros_time(t, Duration)))
        if not self.client.server_is_ready(): raise ValueError('waiting for arm controller')
        current = self.full_state(goal)  # Replan from changed geometry; never abort the whole request.
        if np.max(abs(current[14:21]-curve(0.))) > .02:
            raise ValueError('arm start changed before dispatch')
        if self.now() >= start_at: raise ValueError('planning/dispatch deadline missed before send')
        self.locked_selection = selected or dict(cell_id='XAlign', start_at_s=start_at,
            contact_at_s=start_at+plan['times_s'][-1], contact_xyz_m=plan['alignment_xyz_m'])
        handle = None; pending = None; hand_task = None; intentional_stop = False
        try:
            pending = self.client.send_goal_async(self.replay_goal(tr))
            while not pending.done():
                self.committed_gate(goal)
                if self.now() >= start_at: self.warn_execution('late action acceptance; schedule unchanged')
                await pause(self, .005)
            handle = pending.result()
            if not handle.accepted: raise ControllerFailure('controller rejected trajectory')
            result = handle.get_result_async()
            if selected is not None:
                hand_task = self.executor.create_task(close_during_descent, plan, start_at, self.hand,
                    now=self.now, pause=lambda dt: pause(self, dt), guard=lambda: self.committed_gate(goal),
                    qualified=lambda elapsed: True, strict_timing=False)
            while not result.done() or (hand_task is not None and not hand_task.done()):
                actual = self.committed_gate(goal)
                try:
                    self.log_replay_sample(curve, start_at, actual)
                    error = np.max(abs(actual[7:]-curve(np.clip(self.now()-start_at, 0., plan['times_s'][-1]))))
                    if error > self.config['tracking_tolerance']: self.warn_execution('stored arm trajectory tracking error')
                except Exception as error:
                    self.warn_execution('replay diagnostic error: '+str(error))
                if selected is not None:
                    elapsed = self.now()-start_at
                    self.phase = 'ExecuteGrasp' if elapsed < 0 else ('DescendClose' if elapsed < plan['events']['contact_s'] else 'LiftBrake')
                if self.now() > start_at+plan['times_s'][-1]+3.:
                    self.warn_execution('controller completion overdue; waiting without cancel')
                await pause(self, .01)
            response = result.result()
            if hand_task is not None:
                if hand_task.exception() is not None: self.warn_execution('hand task: '+str(hand_task.exception()))
                else: self.hand_timing = hand_task.result()
            if response.status != 4 or response.result.error_code:
                raise ControllerFailure('controller ended motion: '+response.result.error_string)
            actual = self.committed_gate(goal)
            if np.max(abs(actual[7:]-curve(plan['times_s'][-1]))) > self.config['final_tolerance']:
                self.warn_execution('final joint error; segment completed without cancellation')
        except InterruptedError:
            intentional_stop = True
            raise
        except Exception as error:
            # Unknown controller/transport outcomes must never cause an automatic second motion.
            raise ControllerFailure(str(error)) from error
        finally:
            # Never issue cancel on monitoring, scheduling, logging, or controller-result errors.
            if intentional_stop:
                if pending is not None and handle is None:
                    def stop_late(future):
                        h = future.result()
                        if h.accepted: h.cancel_goal_async()
                    pending.add_done_callback(stop_late)
                await cancel_children(self, [handle])
            if hand_task is not None:
                if not hand_task.done(): hand_task.cancel()
                else: hand_task.exception()  # Retrieve stop errors even when the arm stop won the race.
            self.locked_selection = None

    def stop_guard(self, goal):
        if goal is not None and goal.is_cancel_requested: raise InterruptedError('operator canceled')
        if self.estop or (self.hand is not None and getattr(self.hand, 'estop', False)):
            raise InterruptedError('e-stop active')

    async def attempt(self, goal, report):
        """One pre-dispatch attempt. Input/plan failures are retried by execute."""
        if self.planned_selection is not None:
            return await self.execute_selected(goal, report)
        self.phase = 'XAlign'
        identity, obj, velocity, stamp = self.tracked_observation()
        observed_now = self.now()
        grasp, _ = self.library.grasp_now(obj, velocity, stamp, observed_now)
        self.target_id = identity
        start = self.full_state(goal); pose = self.robot.ee_fk('right', start)
        self.x_error = float(grasp[0, 3]-pose[0, 3])
        upper_error = np.linalg.norm(pose[1:3, 3]-[self.policy['standby_y_m'], self.policy['upper_z_m']])
        if abs(self.x_error) > self.policy['x_deadband_m'] or upper_error > .003:
            planner = RoiPlanner(self.config, self.policy)
            plan = await self.plan(goal, planner.align, start, float(grasp[0, 3]))
            if np.max(abs(self.full_state(goal)-start)) > .02:
                raise ValueError('arm moved during planning; refresh start and replan')
            tid, obj, velocity, stamp = self.tracked_observation()
            fresh, _ = self.library.grasp_now(obj, velocity, stamp, self.now())
            if tid != identity or abs(fresh[0, 3]-grasp[0, 3]) > self.policy['x_deadband_m']:
                raise ValueError('target changed during alignment planning; refresh target')
            self.phase_reason = 'complete upper alignment segment; then refresh observation'
            await self.play(goal, plan, self.now()+self.policy['dispatch_lead_s'])
            return False
        self.phase = 'AwaitGrasp'; self.phase_reason = 'waiting for grasp region and sufficient arrival time'
        planning_policy = dict(self.policy, dispatch_lead_s=self.policy['dispatch_lead_s']+self.policy.get('planning_budget_s', .5))
        selected = encounter(grasp[:3, 3], pose[0, 3], self.library.entries,
                             min(self.policy['descent_durations_s']), observed_now, planning_policy)
        selected.update(target_id=identity, acquired_at_s=stamp, selected_at_s=observed_now)
        self.planned_selection = selected
        report.update(selection=dict(selected), target_id=identity)
        return await self.execute_selected(goal, report)

    async def execute_selected(self, goal, report):
        """Frozen encounter: no perception reads, cell changes or contact-time updates."""
        selected = dict(self.planned_selection)
        self.phase = 'PlanGrasp'; self.phase_reason = 'cell and contact time frozen; perception is diagnostic only'
        start = self.full_state(goal)
        for duration in self.policy['descent_durations_s']:
            start_at = selected['contact_at_s']-duration
            if self.now()+self.policy['dispatch_lead_s'] > start_at:
                self.warn_execution('selected contact has insufficient planning/dispatch time; never shift or reselect')
                continue
            try:
                planner = RoiPlanner(self.config, self.policy)
                plan = await self.plan(goal, planner.grasp, start, selected['contact_xyz_m'], duration)
                if np.max(abs(self.full_state(goal)-start)) > .02:
                    raise ValueError('arm moved during planning; replan same frozen encounter')
                selected.update(start_at_s=start_at, descent_duration_s=duration)
                self.planned_selection = dict(selected)
                report.update(selection=dict(selected), plan=plan)
                self.phase = 'ExecuteGrasp'; self.phase_reason = 'fixed cell, contact time and validated path'
                await self.play(goal, plan, start_at, selected)
                return True
            except ValueError as error:
                self.stop_guard(goal)
                self.phase_reason = str(error); self.warn_execution(str(error))
        self.phase_reason = 'frozen encounter not dispatchable; no retargeting; awaiting explicit cancel'
        return False

    async def execute(self, goal):
        result = MoveToReady.Result(); self.used = True
        self.execution_warnings = []; self.motion_samples = []; self.hand_timing = None
        report = dict(physical_grasp_verified=False, policy=self.policy)
        deadline = time.monotonic()+float(self.get_parameter('target_wait_timeout_s').value)
        try:
            # Readiness fluctuations do not terminate an accepted request.
            while True:
                self.stop_guard(goal)
                try:
                    await self.hand.open_ready(goal); break
                except ValueError as error:
                    self.stop_guard(goal)
                    self.phase = 'OpeningRightHand'; self.phase_reason = str(error)
                    self.warn_execution(str(error)); await pause(self, .1)
            while True:
                self.stop_guard(goal)
                if time.monotonic() > deadline:
                    self.warn_execution('target wait deadline exceeded; continue waiting until explicit cancel')
                try:
                    completed = await self.attempt(goal, report)
                except ValueError as error:
                    self.stop_guard(goal)
                    self.phase_reason = str(error); self.warn_execution(str(error))
                    await pause(self, .05); continue
                if completed:
                    result.success = True; result.reason = 'Scheduled motion completed; physical grasp unverified'
                    self.phase = 'Done'; self.phase_reason = result.reason; break
                await pause(self, .05)
        except Exception as error:
            self.phase = 'Stopped'; result.success = False; result.reason = str(error)
            self.phase_reason = result.reason
        finally:
            report.update(success=result.success, reason=result.reason, warnings=self.execution_warnings,
                          hand_timing=self.hand_timing)
            name = 'run-'+str(time.time_ns())
            try:
                (self.out/(name+'.json')).write_text(json.dumps(report, indent=2))
                (self.out/(name+'.jsonl')).write_text(''.join(json.dumps(r)+'\n' for r in self.motion_samples))
            except OSError as error: self.get_logger().error('cannot save run diagnostics: '+str(error))
        return result


def main():
    rclpy.init(); node = RoiTrial()
    try: rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException): pass
    finally:
        node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__': main()
