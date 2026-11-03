"""Jazzy actual execution action for the new alignment tail; FAKE hardware only.

Run on an available Jazzy host, e.g. ROS_DOMAIN_ID=214 python ... [--cancel].
Never run in hardware domain 12 or 0. Not a physical collision/grasp test.
"""
import os
if os.environ.get('ROS_DOMAIN_ID') in (None,'0','12'):raise RuntimeError('isolated ROS domain required')
import argparse,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient,ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from deux_task_msgs.action import ExecuteMotion
from deux_task_msgs.msg import MotionPlan
from deux_task_core.model import JOINT_NAMES,ARM_NAMES
from deux_task_core.planning import scene_version
from deux_task_runtime.common import ros_time,seconds,pause
from deux_task_runtime.dataset_reading_pose import build_reading_plan,load_pose
from deux_task_runtime.trajectory import sample
import deux_task_runtime.execution_node as execution


class FakeHand:
    mapping={'protocol':'leader_grip'}
    def __init__(self,node):self.node=node;self.commands=[]
    def measured_hand(self):return np.asarray(self.node.config['hand_closed'])
    def validate(self,*args,**kwargs):pass
    def guard(self):pass
    def hold_measured(self):pass
    def command_align_left_fraction(self,*args,**kwargs):pass
    def command(self,positions,placement_right_fraction=None,**kwargs):
        self.commands.append((self.node.now(),kwargs.get('align_left_fraction'),self.node.arm.measured_arm().copy()))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--cancel',action='store_true');args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    rclpy.init(args=['--ros-args','-p','config_file:='+str(root/'src/deux_task_bringup/config/installation.yaml'),
        '-p','hardware_enabled:=true','-p','dry_run:=false','-p','parallel_reader_retreat:=true',
        '-p','driver_action:=/synthetic/parallel_arm','-p','dataset_reading_pose_file:='+str(root/'artifacts/dataset-barcode-pose/reading-pose.json')])
    execution.make_hand_adapter=FakeHand # Only this isolated test process.
    node=execution.ExecutionNode();node.hardware_gate=lambda:None
    source=Node('parallel_fake_hardware');source.group=ReentrantCallbackGroup()
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(source)
    c=node.config;pose=load_pose(root/'artifacts/dataset-barcode-pose/reading-pose.json');start=np.asarray(c['home_joints']).copy();start[14:21]=c['scan_right_joints'];start[:7]=pose['left_joints_rad']
    data=build_reading_plan(c,pose,start);arm_idx=np.r_[np.arange(7),np.arange(14,21)]
    active={};requests=[];cancellations=[]
    async def arm(goal):
        tr=goal.request.trajectory;requests.append(tr)
        assert list(tr.joint_names)==list(ARM_NAMES),'must command both arms in one trajectory'
        active.update(start=seconds(tr.header.stamp),times=np.asarray([seconds(p.time_from_start) for p in tr.points]),path=np.asarray([p.positions for p in tr.points]))
        end=active['start']+active['times'][-1]
        while node.now()<end:
            if goal.is_cancel_requested:
                cancellations.append(True);goal.canceled();return FollowJointTrajectory.Result(error_code=0)
            await pause(source,.005)
        goal.succeed();return FollowJointTrajectory.Result(error_code=0)
    server=ActionServer(source,FollowJointTrajectory,'/synthetic/parallel_arm',arm,
        cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=source.group)
    publisher=source.create_publisher(JointState,'/arm/joint_states',qos_profile_sensor_data)
    def publish():
        q=start[arm_idx]
        if active:q=sample(active['times'],active['path'],np.clip(node.now()-active['start'],0,active['times'][-1]))
        msg=JointState(name=list(ARM_NAMES),position=q.tolist());msg.header.stamp=ros_time(node.now());publisher.publish(msg)
    timer=source.create_timer(.005,publish)
    client=ActionClient(source,ExecuteMotion,'/task/execute_motion')
    def spin(predicate,timeout=10.):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        assert predicate(),'synthetic ROS test timed out'
    try:
        spin(lambda:client.server_is_ready() and node.arm.client.server_is_ready() and node.arm.state is not None)
        plan=MotionPlan(stage='AlignBarcode',plan_id='yaw-tail-fake',scene_version=scene_version(c),model_id=c['model_id'],ik_backend=c['ik_backend'])
        plan.header.stamp=ros_time(node.now());plan.header.frame_id='base_link';plan.valid_until=ros_time(node.now()+30.)
        plan.trajectory.joint_names=list(JOINT_NAMES)
        plan.trajectory.points=[JointTrajectoryPoint(positions=q.tolist(),time_from_start=ros_time(float(t),Duration)) for t,q in zip(data['times'],data['path'])]
        pending=client.send_goal_async(ExecuteMotion.Goal(plan=plan));spin(pending.done);handle=pending.result();assert handle.accepted
        result=handle.get_result_async()
        if args.cancel:
            spin(lambda:active and node.now()>active['start']+.2)
            cancel=handle.cancel_goal_async();spin(cancel.done)
        spin(result.done,15.);response=result.result();assert len(requests)==1,response.result.reason
        if args.cancel:
            assert response.status==5 and cancellations
            assert active['times'][-1]>node.now()-active['start'],'cancellation happened after completion'
        else:
            assert response.status==4 and response.result.success,response.result.reason
            closed=[(stamp-active['start'],q) for stamp,fraction,q in node.hand.commands if fraction is not None and fraction>=1.]
            assert closed and closed[0][0]<data['alignment_tail_start_s']
            np.testing.assert_allclose(closed[0][1][:7],pose['left_joints_rad'],atol=1e-8)
            np.testing.assert_allclose(requests[0].points[-1].positions[:7],pose['alignment_tail']['target_left_joints_rad'],atol=1e-8)
            np.testing.assert_allclose(np.array([p.positions for p in requests[0].points])[:,7:],np.tile(start[14:21],(len(requests[0].points),1)),atol=1e-12)
        print('PASS: synthetic AlignBarcode '+('cancellation' if args.cancel else 'hand closed before base-yaw tail; right arm held; endpoint reached'))
    finally:
        source.destroy_timer(timer);server.destroy();node.destroy_node();source.destroy_node();executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
