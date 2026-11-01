"""Isolated ROS replay against fake controller/hand, never robot hardware.

Tests the actual execution coroutine and concurrent hand scheduler. Hardware
mode/profile checks are replaced ONLY in this fixture. Domain 12 is forbidden.
"""
import os
if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '12'):
    raise RuntimeError('Use isolated ROS_DOMAIN_ID=218, never hardware domain 12')
import json
import sys
import time
import types
import argparse
import numpy as np
from scipy.interpolate import CubicHermiteSpline
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from deux_task_msgs.msg import ObjectDetectionArray, ObjectDetection
from deux_task_msgs.action import MoveToReady
from deux_task_core.model import ARM_NAMES
from deux_task_runtime.common import pause, seconds, ros_time


class FakeHand:
    def __init__(self, node): self.node = node; self.commands = []
    def guard(self): pass
    def send(self, fraction): self.commands.append((self.node.now(), fraction))
    async def open_ready(self, goal): await pause(self.node, .02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--roi', action='store_true')
    parser.add_argument('--rotation-grid', action='store_true')
    parser.add_argument('--classified-grid', action='store_true')
    parser.add_argument('--object-yaw',type=float,default=0.)
    parser.add_argument('--object-x',type=float,default=.48)
    parser.add_argument('--velocity-noise', action='store_true')
    parser.add_argument('--transient-input', action='store_true')
    parser.add_argument('--short-wait', action='store_true')
    parser.add_argument('--stop-fault', action='store_true')
    parser.add_argument('--selection-vision-fault', action='store_true')
    parser.add_argument('--vision-fault', action='store_true')
    parser.add_argument('--arm-tracking-fault', action='store_true')
    parser.add_argument('--alignment-bias', action='store_true')
    options = parser.parse_args()
    sys.modules['right_trial_hand'] = types.SimpleNamespace(RightTrialHand=FakeHand)
    if options.classified_grid:
        from classified_grasp_trial import ClassifiedTrial as GridTrial
    elif options.roi:
        from roi_grasp_trial import RoiTrial as GridTrial
    else:
        from grid_grasp_trial import GridTrial
    ros_args = ['--ros-args', '-p', 'enable_execution:=true']
    if options.rotation_grid or options.classified_grid:
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        directory='right_grasp_grid_classified' if options.classified_grid else 'right_grasp_grid_rotations'
        ros_args += ['-p','grid_library_directory:='+str(root/'artifacts'/directory),
                     '-p','use_observed_yaw:=true','-p','recommended_only:=false']
    if options.short_wait: ros_args += ['-p', 'target_wait_timeout_s:=0.01']
    rclpy.init(args=ros_args)
    runtime = GridTrial()
    injected = [False]; selected_snapshot = []
    if options.transient_input or options.selection_vision_fault:
        original_observation = runtime.tracked_observation
        def intermittent_observation():
            if options.selection_vision_fault and runtime.planned_selection is not None and sys._getframe(1).f_code.co_name in ('attempt', 'execute_selected'):
                raise AssertionError('perception read after encounter selection')
            if options.transient_input and runtime.action.busy and runtime.x_error is not None and not injected[0] and sys._getframe(1).f_code.co_name == 'attempt':
                injected[0] = True
                raise ValueError('injected observation gap between planning and dispatch')
            return original_observation()
        runtime.tracked_observation = intermittent_observation
    runtime.out = runtime.out.parent/'grid-grasp-tests/synthetic-execution'; runtime.out.mkdir(parents=True, exist_ok=True)
    # Synthetic only: real mode, hardware ownership and hand profile checks are
    # exercised by live admission, not by this fake-controller timing fixture.
    runtime.hardware_gate = lambda: None
    producer = Node('grid_fake_hardware_test'); producer.group = ReentrantCallbackGroup()
    executor = SingleThreadedExecutor(); executor.add_node(runtime); executor.add_node(producer)
    active = {}; received = []; cancellations = []
    async def arm_execute(goal):
        tr = goal.request.trajectory
        assert tr.joint_names == [f'arm_r_joint{i}' for i in range(1, 8)]
        assert all(t.position == -1. and t.velocity == -1. and t.acceleration == -1. for t in goal.request.path_tolerance)
        assert len(goal.request.path_tolerance) == 7
        times = [seconds(p.time_from_start) for p in tr.points]
        active.update(start=seconds(tr.header.stamp), curve=CubicHermiteSpline(times, [p.positions for p in tr.points], [p.velocities for p in tr.points]))
        while runtime.now() < active['start']+times[-1]:
            if goal.is_cancel_requested:
                cancellations.append(True)
                goal.canceled(); return FollowJointTrajectory.Result(error_code=0)
            await pause(producer, .005)
        goal.succeed(); return FollowJointTrajectory.Result(error_code=0)
    server = ActionServer(producer, FollowJointTrajectory, '/arm/deux_arm_controller/follow_joint_trajectory',
                          arm_execute, cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=producer.group)
    predictions = producer.create_publisher(ObjectDetectionArray, '/task/prediction_detections', qos_profile_sensor_data)
    arms = producer.create_publisher(JointState, '/arm/joint_states', qos_profile_sensor_data)
    tracking = producer.create_publisher(String, '/task/icp_preview_status', 10)
    calibration = producer.create_publisher(String, '/task/camera_calibration_status', 10)
    estop = producer.create_publisher(Bool, '/deux/estop', 10)
    began = runtime.now(); last_prediction = [-float('inf')]
    def publish():
        now = runtime.now()
        elapsed = now-active['start'] if active else -1.
        if options.roi and (runtime.locked_selection or {}).get('cell_id', 'XAlign') == 'XAlign': elapsed = -1.
        if now-last_prediction[0] >= .04:
            last_prediction[0] = now
            calibration.publish(String(data=json.dumps(dict(valid=True))))
            estop.publish(Bool(data=bool(options.stop_fault and elapsed >= .2)))
            tracking.publish(String(data=json.dumps(dict(tracking=True, target_id='execution-synthetic'))))
            msg = ObjectDetectionArray(); msg.header.frame_id = 'base_link'; msg.header.stamp = ros_time(now)
            msg.model_id = runtime.config['model_id']; msg.sequence = round(now*1000)
            d = ObjectDetection(); d.track_id = 'execution-synthetic'; d.confidence = 1.; d.velocity_valid = True
            d.velocity.linear.y = -.09; d.pose.pose.orientation.w = 1.
            d.pose.pose.orientation.z=float(np.sin(np.deg2rad(options.object_yaw)/2.))
            d.pose.pose.orientation.w=float(np.cos(np.deg2rad(options.object_yaw)/2.))
            d.pose.pose.position.x = options.object_x; d.pose.pose.position.y = .48-.09*(now-began); d.pose.pose.position.z = .97
            if options.vision_fault and elapsed >= .2:
                d.pose.pose.position.x += .15; d.pose.pose.position.y += .20
                if elapsed >= .6:
                    d.track_id = 'changed-identity'
                    tracking.publish(String(data=json.dumps(dict(tracking=elapsed < 1., target_id='changed-identity'))))
            if options.selection_vision_fault and runtime.planned_selection is not None:
                if not selected_snapshot: selected_snapshot.append(dict(runtime.planned_selection))
                tracking.publish(String(data=json.dumps(dict(tracking=False, target_id='changed-after-selection'))))
                msg.detections = []
            else:
                msg.detections = [] if options.vision_fault and elapsed >= 1. else [d]
            predictions.publish(msg)
        arm = JointState(); arm.header.stamp = ros_time(now); arm.name = list(ARM_NAMES)
        q = runtime.library.ready[14:21]; v = np.zeros(7)
        if active:
            t = np.clip(now-active['start'], 0., active['curve'].x[-1]); q = active['curve'](t); v = active['curve'](t, 1)
        if options.arm_tracking_fault and elapsed >= .2:
            q = np.array(q).copy(); q[0] += .35
        elif options.alignment_bias and elapsed >= .2:
            q = np.array(q).copy(); q[0] += .05
        if options.velocity_noise: v = np.asarray(v)+.12
        arm.position = [*runtime.library.ready[:7], *q]; arm.velocity = [0.]*7+list(v); arms.publish(arm)
    timer = producer.create_timer(.005, publish, callback_group=producer.group)
    try:
        deadline = time.monotonic()+4
        while not runtime.preview.get('selectable') and time.monotonic() < deadline: executor.spin_once(timeout_sec=.005)
        assert runtime.preview.get('selectable'), runtime.preview
        client = ActionClient(producer, MoveToReady, '/task/grid_grasp')
        pending = client.send_goal_async(MoveToReady.Goal()); deadline = time.monotonic()+3
        while not pending.done() and time.monotonic() < deadline: executor.spin_once(timeout_sec=.005)
        assert pending.done() and pending.result().accepted, 'synthetic execution request rejected'
        result_future = pending.result().get_result_async(); deadline = time.monotonic()+25
        while not result_future.done() and time.monotonic() < deadline: executor.spin_once(timeout_sec=.005)
        assert result_future.done(), 'synthetic replay timed out'
        response = result_future.result()
        if options.stop_fault:
            assert not response.result.success and 'e-stop' in response.result.reason, response.result.reason
            assert cancellations, 'explicit stop did not cancel the controller goal'
            print('PASS: explicit e-stop still cancels the committed trajectory')
            return
        assert not cancellations, 'monitoring error canceled the trajectory'
        if options.arm_tracking_fault:
            assert any('tracking error' in w for w in runtime.execution_warnings), runtime.execution_warnings
            assert any(row.get('max_joint_error_rad', 0.) > .3 for row in runtime.motion_samples)
        assert response.result.success, response.result.reason
        if options.velocity_noise: assert any('velocity' in w for w in runtime.execution_warnings)
        if options.transient_input: assert injected[0] and any('injected observation gap' in w for w in runtime.execution_warnings)
        if options.short_wait: assert any('deadline exceeded' in w for w in runtime.execution_warnings)
        if options.selection_vision_fault:
            assert selected_snapshot
            for key in ('cell_id', 'contact_at_s', 'target_id', 'contact_xyz_m'):
                assert runtime.planned_selection[key] == selected_snapshot[0][key], key
            assert any(not row.get('vision_available', True) for row in runtime.motion_samples)
        assert len(runtime.motion_samples) >= 20, 'replay diagnostics must be retained'
        assert any('cartesian_path_error_m' in row for row in runtime.motion_samples)
        if options.vision_fault:
            assert any(not row.get('vision_available', True) for row in runtime.motion_samples)
            assert any(row.get('prediction_xy_error_m', 0.) > .03 for row in runtime.motion_samples)
        commands = [(stamp-active['start'], fraction) for stamp, fraction in runtime.hand.commands]
        contact = active['curve'].x[-1]-2. if options.roi else 3.
        assert commands and abs(commands[0][0]-(contact-.5)) <= .06, commands[:1]
        assert abs(commands[-1][0]-contact) <= .06, commands[-1:]
        assert all(fraction == 1. for stamp, fraction in commands)
        print(json.dumps(dict(synthetic_only=True, fake_hand=True, fake_controller=True,
                             action_success=response.result.success, first_close_s=commands[0][0],
                             last_close_s=commands[-1][0], hand_commands=len(commands), reason=response.result.reason), indent=2))
        if options.vision_fault: print('PASS: XY drift, changed target ID and complete tracking loss did not cancel stored replay')
        if options.alignment_bias:
            assert runtime.max_close_path_error_m > .015, runtime.max_close_path_error_m
            print('PASS: Cartesian alignment is diagnostic; joint tracking errors are diagnostic. max_error_m='+str(runtime.max_close_path_error_m))
    finally:
        producer.destroy_timer(timer); server.destroy(); executor.remove_node(runtime); executor.remove_node(producer)
        if options.roi: runtime.pool.shutdown(wait=False, cancel_futures=True)
        runtime.destroy_node(); producer.destroy_node(); executor.shutdown(); rclpy.shutdown()


if __name__ == '__main__': main()
