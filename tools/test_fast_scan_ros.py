"""Jazzy: actual LookupBtServer, one compiled candidate, fake arm and hand only."""
import os
if os.environ.get('ROS_DOMAIN_ID') in (None,'0','12'):raise RuntimeError('isolated ROS domain required')
import sys,time,types,json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient,ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from deux_task_msgs.action import LookupGrasp,MoveToReady
from deux_task_core.model import ARM_NAMES
from deux_task_runtime.common import pause,ros_time,seconds

class FakeHand:
    def __init__(self,node):self.node=node;self.commands=[]
    def guard(self):pass
    async def open_ready(self,goal):await pause(self.node,.02)
    def send(self,fraction):self.commands.append((self.node.now(),fraction))


def main():
    root=Path(__file__).resolve().parents[1]
    sys.modules['right_trial_hand']=types.SimpleNamespace(RightTrialHand=FakeHand)
    import lookup_lift_scan
    # Limit THIS isolated fixture to one real candidate. Production compiles all.
    key='x+021_y-001_axis09_29'
    from lookup_bt_server import LookupBtServer
    rclpy.init(args=['--ros-args','-p','enable_execution:=true',
        '-p','grid_library_directory:='+str(root/'artifacts/right_grasp_grid_classified'),
        '-p','stationary_left_file:='+str(root/'.setup/lookup-bt/home-left-conveyor.json'),
        '-p','dataset_reading_pose_file:='+str(root/'artifacts/dataset-barcode-pose/reading-pose.json'),
        '-p','config_file:='+str(root/'src/deux_task_bringup/config/installation.yaml'),
        '-p','scan_timing_profile:=via_preserved_fast_scan',
        '-p','use_observed_yaw:=true','-p','recommended_only:=false',
        '-p','grid_output_directory:='+str(root/'.setup/lookup-bt/deployment-validation/parallel-scan')])
    runtime=LookupBtServer();runtime.hardware_gate=lambda:None
    lookup_lift_scan.restrict_selection(runtime.library,{key})
    producer=Node('parallel_scan_fake_hardware');producer.group=ReentrantCallbackGroup()
    executor=SingleThreadedExecutor();executor.add_node(runtime);executor.add_node(producer)
    start=runtime.now();active={};received=[]
    def observation():
        now=runtime.now();obj=np.eye(4);obj[:3,3]=[.63,.50-.09*(now-start),.97]
        return 'parallel-scan-synthetic',obj,np.array([0.,-.09,0.,0.]),now
    runtime.tracked_observation=observation
    async def arm_execute(goal):
        tr=goal.request.trajectory;received.append(tr)
        assert list(tr.joint_names)==list(ARM_NAMES)
        assert len(goal.request.path_tolerance)==14
        ts=np.array([seconds(p.time_from_start) for p in tr.points]);qs=np.array([p.positions for p in tr.points]);vs=np.array([p.velocities for p in tr.points])
        active.update(start=seconds(tr.header.stamp),curve=CubicHermiteSpline(ts,qs,vs))
        while runtime.now()<active['start']+ts[-1]:
            if goal.is_cancel_requested:goal.canceled();return FollowJointTrajectory.Result(error_code=0)
            await pause(producer,.005)
        goal.succeed();return FollowJointTrajectory.Result(error_code=0)
    server=ActionServer(producer,FollowJointTrajectory,'/arm/deux_arm_controller/follow_joint_trajectory',arm_execute,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=producer.group)
    publisher=producer.create_publisher(JointState,'/arm/joint_states',qos_profile_sensor_data)
    def publish():
        q=np.r_[runtime.library.ready[:7],runtime.library.ready[14:21]]
        if active:q=active['curve'](np.clip(runtime.now()-active['start'],0.,active['curve'].x[-1]))
        msg=JointState(name=list(ARM_NAMES),position=q.tolist());msg.header.stamp=ros_time(runtime.now());publisher.publish(msg)
    timer=producer.create_timer(.005,publish)
    def spin(predicate,timeout=12.):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        assert predicate(),'parallel scan fixture timeout'
    prepare=ActionClient(producer,MoveToReady,'/task/lookup_prepare');client=ActionClient(producer,LookupGrasp,'/task/lookup_grasp')
    def call(action,request,timeout=12.):
        pending=action.send_goal_async(request);spin(pending.done);handle=pending.result();assert handle.accepted
        done=handle.get_result_async();spin(done.done,timeout);response=done.result();assert response.status==4 and response.result.success,response.result.reason
        return response.result
    try:
        spin(lambda:prepare.server_is_ready() and client.server_is_ready() and runtime.client.server_is_ready() and runtime.arm is not None)
        call(prepare,MoveToReady.Goal(),90.)
        lookup_lift_scan.restrict_selection(runtime.library,{key})
        start=runtime.now()
        result=call(client,LookupGrasp.Goal(request_id='parallel-scan-fixture',scene_version=runtime.config_version,target_id='parallel-scan-synthetic'),20.)
        assert len(received)==1 and 'left_scan_completed' in result.reason
        tr=received[0];ts=np.array([seconds(p.time_from_start) for p in tr.points]);q=np.array([p.positions for p in tr.points]);expected=np.array(runtime.parallel_scan_left)
        np.testing.assert_allclose(q[ts<=6.,:7],np.tile(runtime.library.ready[:7],(sum(ts<=6.),1)),atol=1e-8)
        np.testing.assert_allclose(q[ts>=7.8,:7],np.tile(expected,(sum(ts>=7.8),1)),atol=1e-8)
        assert np.max(abs(q[np.argmin(abs(ts-6.9)),:7]-q[0,:7]))>.1
        commands=np.array(runtime.hand.commands);elapsed=commands[:,0]-active['start'];assert abs(elapsed[0]-2.5)<.08 and abs(elapsed[-1]-3.)<.08
        assert any('planned_left_rad' in row for row in runtime.motion_samples)
        assert not runtime.execution_warnings,runtime.execution_warnings
        print(json.dumps(dict(PASS=True,synthetic_only=True,joints=14,contact_s=3.,left_moves_s=[6.,7.8],scan_s=float(ts[-1]),first_close_s=float(elapsed[0]),last_close_s=float(elapsed[-1])),indent=2))
    finally:
        producer.destroy_timer(timer);server.destroy();runtime.pool.shutdown(wait=False,cancel_futures=True)
        runtime.destroy_node();producer.destroy_node();executor.shutdown();rclpy.shutdown()

if __name__=='__main__':main()
