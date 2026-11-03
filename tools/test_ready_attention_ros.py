"""Isolated Jazzy: real prepare/return actions and fake arm controller only."""
import os
if os.environ.get('ROS_DOMAIN_ID') in (None,'0','12'):raise RuntimeError('isolated domain required')
import sys,time,json,types
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
from deux_task_msgs.action import MoveToReady
from deux_task_core.model import ARM_NAMES
from deux_task_runtime.common import pause,ros_time,seconds

class FakeHand:
 def __init__(self,node):self.commands=[]
 def guard(self):pass

def main():
 root=Path(__file__).resolve().parents[1]
 sys.modules['right_trial_hand']=types.SimpleNamespace(RightTrialHand=FakeHand)
 from lookup_bt_server import LookupBtServer
 from lookup_bt_node import LookupTaskNode
 rclpy.init(args=['--ros-args','-p','config_file:='+str(root/'src/deux_task_bringup/config/installation.yaml'),
  '-p','enable_execution:=true','-p','grid_library_directory:='+str(root/'artifacts/right_grasp_grid_classified'),
  '-p','stationary_left_file:='+str(root/'.setup/lookup-bt/home-left-conveyor.json'),
  '-p','dataset_reading_pose_file:='+str(root/'artifacts/dataset-barcode-pose/reading-pose.json'),
  '-p','use_observed_yaw:=true','-p','scan_timing_profile:=via_preserved_fast_scan',
  '-p','grid_output_directory:=/tmp/deux-motion-synthetic','-p','bt_output_directory:=/tmp/deux-motion-synthetic'])
 runtime=LookupBtServer();runtime.hardware_gate=lambda:None
 coordinator=LookupTaskNode();coordinator.robot_ready=lambda:None
 fake=Node('ready_attention_fake_arm');fake.group=ReentrantCallbackGroup()
 executor=SingleThreadedExecutor()
 for node in [runtime,coordinator,fake]:executor.add_node(node)
 active={};received=[];actual=np.zeros(14)
 async def arm_execute(goal):
  tr=goal.request.trajectory;received.append(tr)
  ts=np.array([seconds(p.time_from_start) for p in tr.points]);qs=np.array([p.positions for p in tr.points]);vs=np.array([p.velocities for p in tr.points])
  active.update(start=seconds(tr.header.stamp),curve=CubicHermiteSpline(ts,qs,vs))
  while runtime.now()<active['start']+ts[-1]:
   if goal.is_cancel_requested:goal.canceled();return FollowJointTrajectory.Result(error_code=0)
   await pause(fake,.005)
  goal.succeed();return FollowJointTrajectory.Result(error_code=0)
 server=ActionServer(fake,FollowJointTrajectory,'/arm/deux_arm_controller/follow_joint_trajectory',arm_execute,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=fake.group)
 pub=fake.create_publisher(JointState,'/arm/joint_states',qos_profile_sensor_data)
 def publish():
  nonlocal actual
  if active:actual=active['curve'](np.clip(runtime.now()-active['start'],0,active['curve'].x[-1]))
  msg=JointState(name=list(ARM_NAMES),position=actual.tolist());msg.header.stamp=ros_time(runtime.now());pub.publish(msg)
 timer=fake.create_timer(.01,publish)
 def spin(predicate,timeout=30.):
  end=time.monotonic()+timeout
  while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.005)
  assert predicate(),'synthetic ready/attention timeout'
 ready=ActionClient(fake,MoveToReady,'/task/lookup_prepare');attention=ActionClient(fake,MoveToReady,'/task/return_to_attention')
 def call(client):
  pending=client.send_goal_async(MoveToReady.Goal());spin(pending.done);handle=pending.result();assert handle.accepted
  result=handle.get_result_async();spin(result.done,60.);response=result.result();assert response.status==4 and response.result.success,response.result.reason
 try:
  spin(lambda:ready.server_is_ready() and attention.server_is_ready() and coordinator.attention_client.server_is_ready() and runtime.arm is not None)
  call(ready);assert len(received)==1
  forward=np.array([p.positions for p in received[0].points]);assert np.max(abs(forward[1,:7]-forward[0,:7]))>0
  assert np.max(abs(forward[1,7:]-forward[0,7:]))>0
  call(attention);assert len(received)==2
  reverse=np.array([p.positions for p in received[1].points]);np.testing.assert_allclose(reverse,forward[::-1],atol=1e-8)
  assert np.max(abs(actual))<.03 and coordinator.shutdown_requested
  assert not runtime.hand.commands
  print(json.dumps(dict(PASS=True,synthetic_only=True,forward_s=seconds(received[0].points[-1].time_from_start),reverse_s=seconds(received[1].points[-1].time_from_start),no_hand_commands=True,scan_profile=runtime.get_parameter('scan_timing_profile').value)))
 finally:
  fake.destroy_timer(timer);server.destroy();runtime.pool.shutdown(wait=False,cancel_futures=True)
  for node in [runtime,coordinator,fake]:node.destroy_node()
  executor.shutdown();rclpy.shutdown()
if __name__=='__main__':main()
