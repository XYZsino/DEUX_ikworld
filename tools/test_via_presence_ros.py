"""Jazzy: actual LookupBtServer, one compiled candidate, fake arm and hand only."""
import os
if os.environ.get('ROS_DOMAIN_ID') in (None,'0','12'):raise RuntimeError('isolated ROS domain required')
import sys,time,types,json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline,BPoly
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient,ActionServer,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from deux_task_msgs.action import LookupGrasp,MoveToReady
from deux_task_core.model import ARM_NAMES,HAND_NAMES
from deux_task_runtime.common import pause,ros_time,seconds

class FakeHand:
    def __init__(self,node):self.node=node;self.commands=[]
    def guard(self):pass
    async def open_ready(self,goal):await pause(self.node,.02)
    def send(self,fraction):self.commands.append((self.node.now(),fraction))


def main():
    slow_archive="--slow-archive" in sys.argv; fail_archive="--fail-archive" in sys.argv
    cancel_brake="--cancel-during-brake" in sys.argv; reject_brake="--reject-brake" in sys.argv
    empty="--empty" in sys.argv; background="--background" in sys.argv; four="--four-frames" in sys.argv; reject=empty or background or four
    from lookup_bt_node import LookupTaskNode,ScanObjectMissing
    from sensor_msgs.msg import CameraInfo,Image
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import StaticTransformBroadcaster
    from deux_task_msgs.msg import DetectionArray,BoundingBox
    root=Path(__file__).resolve().parents[1]
    sys.modules['right_trial_hand']=types.SimpleNamespace(RightTrialHand=FakeHand)
    import lookup_lift_scan
    # Limit THIS isolated fixture to one real candidate. Production compiles all.
    key='x+021_y-001_axis09_29'
    from lookup_bt_server import LookupBtServer
    rclpy.init(args=['--ros-args','-p','enable_execution:=true','-p','via_frame_directory:=/tmp/via-mask-test-'+os.environ['ROS_DOMAIN_ID'],
        '-p','grid_library_directory:='+str(root/'artifacts/right_grasp_grid_classified'),
        '-p','stationary_left_file:='+str(root/'.setup/lookup-bt/home-left-conveyor.json'),
        '-p','dataset_reading_pose_file:='+str(root/'artifacts/dataset-barcode-pose/reading-pose.json'),
        '-p','config_file:='+str(root/'src/deux_task_bringup/config/installation.yaml'),
        '-p','scan_timing_profile:=via_preserved_fast_scan',
        '-p','use_observed_yaw:=true','-p','recommended_only:=false',
        '-p','grid_output_directory:='+str(root/'.setup/lookup-bt/deployment-validation/parallel-scan')])
    runtime=LookupBtServer();runtime.hardware_gate=lambda:None
    coordinator=LookupTaskNode();coordinator.guard=lambda *a:None
    if slow_archive:
        import via_frame_archive
        original_save=via_frame_archive.np.save
        def slow_save(filename,array,*args,**kwargs):
            time.sleep(.35)
            return original_save(filename,array,*args,**kwargs)
        via_frame_archive.np.save=slow_save
    if fail_archive:
        def failed_save(*args,**kwargs):raise OSError('synthetic archive unavailable')
        coordinator.via_depth.archive.save=failed_save
    async def hold(value):pass
    coordinator.hold_perception=hold
    lookup_lift_scan.restrict_selection(runtime.library,{key})
    producer=Node('parallel_scan_fake_hardware');producer.group=ReentrantCallbackGroup()
    executor=SingleThreadedExecutor();executor.add_node(runtime);executor.add_node(producer);executor.add_node(coordinator)
    start=runtime.now();active={};received=[];canceled=[]
    def observation():
        now=runtime.now();obj=np.eye(4);obj[:3,3]=[.63,.50-.09*(now-start),.97]
        return 'parallel-scan-synthetic',obj,np.array([0.,-.09,0.,0.]),now
    runtime.tracked_observation=observation
    async def arm_execute(goal):
        tr=goal.request.trajectory;received.append(tr)
        assert list(tr.joint_names)==list(ARM_NAMES)
        assert len(goal.request.path_tolerance)==14 or len(received)>1
        ts=np.array([seconds(p.time_from_start) for p in tr.points]);qs=np.array([p.positions for p in tr.points]);vs=np.array([p.velocities for p in tr.points])
        curve=BPoly.from_derivatives(ts,[[q,v,p.accelerations] for q,v,p in zip(qs,vs,tr.points)]) if tr.points[0].accelerations else CubicHermiteSpline(ts,qs,vs)
        if reject_brake and len(received)==2:
            goal.abort();return FollowJointTrajectory.Result(error_code=-1,error_string='synthetic brake controller fault')
        active.clear();active.update(start=seconds(tr.header.stamp),curve=curve,goal=goal)
        while runtime.now()<seconds(tr.header.stamp)+ts[-1]:
            if active.get('goal') is not goal:
                goal.abort();return FollowJointTrajectory.Result(error_code=0)
            if cancel_brake and len(received)==2 and runtime.now()>seconds(tr.header.stamp)+.15:
                runtime.estop=True
            if goal.is_cancel_requested:
                elapsed=runtime.now()-active['start'];active['held']=active['curve'](elapsed);canceled.append(elapsed)
                goal.canceled();return FollowJointTrajectory.Result(error_code=0)
            await pause(producer,.005)
        goal.succeed();return FollowJointTrajectory.Result(error_code=0)
    server=ActionServer(producer,FollowJointTrajectory,'/arm/deux_arm_controller/follow_joint_trajectory',arm_execute,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=producer.group)
    publisher=producer.create_publisher(JointState,'/arm/joint_states',qos_profile_sensor_data)
    hand_pub=producer.create_publisher(JointState,'/task/hand_state',qos_profile_sensor_data)
    def publish():
        q=np.r_[runtime.library.ready[:7],runtime.library.ready[14:21]]
        if active:q=active['held'] if 'held' in active else active['curve'](np.clip(runtime.now()-active['start'],0.,active['curve'].x[-1]))
        msg=JointState(name=list(ARM_NAMES),position=q.tolist());msg.header.stamp=ros_time(runtime.now());publisher.publish(msg)
        hand_msg=JointState(header=msg.header,name=list(HAND_NAMES),position=np.r_[runtime.library.ready[7:14],runtime.library.ready[21:28]].tolist());hand_pub.publish(hand_msg)
    timer=producer.create_timer(.005,publish)
    info_pub=producer.create_publisher(CameraInfo,'/zed/color/camera_info',qos_profile_sensor_data)
    boxes_pub=producer.create_publisher(DetectionArray,'/task/detections',qos_profile_sensor_data)
    depth_pub=producer.create_publisher(Image,'/zed/depth/image_raw',qos_profile_sensor_data)
    rgb_pub=producer.create_publisher(Image,'/zed/color/image_raw',qos_profile_sensor_data)
    depth_info_pub=producer.create_publisher(CameraInfo,'/zed/depth/camera_info',qos_profile_sensor_data)
    broadcaster=StaticTransformBroadcaster(producer)
    tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id='fake_rgb';tf.transform.rotation.w=1.;broadcaster.sendTransform(tf)
    queued=[];sent=[0]
    def camera():
        if queued:boxes_pub.publish(queued.pop(0))
        info=CameraInfo(width=640,height=360,k=[500.,0.,320.,0.,500.,180.,0.,0.,1.],p=[500.,0.,320.,0.,0.,500.,180.,0.,0.,0.,1.,0.]);info.header.frame_id='fake_rgb';info.header.stamp=ros_time(runtime.now())
        info_pub.publish(info);depth_info_pub.publish(info)
        q=runtime.library.ready.copy()
        if active:
            arms=active['held'] if 'held' in active else active['curve'](np.clip(runtime.now()-active['start'],0.,active['curve'].x[-1]))
            q[:7]=arms[:7];q[14:21]=arms[7:]
        z=float(runtime.robot.ee_fk('right',q)[2,3])+(0.4 if background else 0.)
        depth_pub.publish(Image(header=info.header,width=640,height=360,encoding='32FC1',step=640*4,data=np.full((360,640),z,dtype='<f4').tobytes()))
        rgb_pub.publish(Image(header=info.header,width=640,height=360,encoding='rgb8',step=640*3,data=bytes(640*360*3)))
        req=coordinator.perception_request
        if req is None:return
        msg=DetectionArray(request_id=req.request_id);msg.header=info.header
        if not empty and (not four or sent[0]<4):
            msg.boxes=[BoundingBox(class_name='bread_bag',confidence=.9,xyxy=[447.,96.,549.,192.],track_id='bag')];sent[0]+=1
        queued.append(msg)
    cam_timer=producer.create_timer(.03,camera)
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
        goal=types.SimpleNamespace(is_cancel_requested=False)
        request=LookupGrasp.Goal(request_id='via-fixture',scene_version=runtime.config_version,target_id='parallel-scan-synthetic')
        task=executor.create_task(coordinator.child_call,client,request,goal,coordinator.config_version,True)
        spin(task.done,25.)
        print('GATE_EVIDENCE',json.dumps(coordinator.scan_bbox_evidence),flush=True)
        if cancel_brake or reject_brake:
            assert task.exception() is not None
            assert not isinstance(task.exception(),ScanObjectMissing),repr(task.exception())
            assert 'soft_stop' not in runtime.last_replay_report
            assert canceled,canceled
            assert len(received)==2  # Recovery must not be dispatched after braking failure.
        elif reject:
            assert isinstance(task.exception(),ScanObjectMissing),repr(task.exception())
            assert not canceled,canceled
            assert len(received)==2
            braking=received[-1];original=received[0]
            offset=seconds(braking.header.stamp)-seconds(original.header.stamp)
            source=CubicHermiteSpline([seconds(p.time_from_start) for p in original.points],
                                     [p.positions for p in original.points],[p.velocities for p in original.points])
            np.testing.assert_allclose(braking.points[0].positions,source(offset),atol=1e-6)
            np.testing.assert_allclose(braking.points[0].velocities,source(offset,1),atol=1e-6)
            np.testing.assert_allclose(braking.points[-1].velocities,np.zeros(14),atol=1e-10)
            assert runtime.last_replay_report['soft_stop']['duration_s']>=.4
            empty_return=ActionClient(producer,MoveToReady,'/task/lookup_empty_return')
            spin(empty_return.server_is_ready)
            call(empty_return,MoveToReady.Goal(),60.)
            assert all(len(p.accelerations)==14 for p in received[-1].points)
            assert len(received)==3
            np.testing.assert_allclose(active['curve'](active['curve'].x[-1]),np.r_[runtime.library.ready[:7],runtime.library.ready[14:21]],atol=1e-8)
        else:
            assert task.exception() is None,repr(task.exception())
            assert 'left_scan_completed' in task.result().reason
            assert not canceled and len(received)==1
            assert seconds(received[0].points[-1].time_from_start)==7.8
        assert coordinator.scan_bbox_evidence['passed']==(not reject)
        if slow_archive:assert coordinator.scan_bbox_evidence['archive_status']['pending']>0
        if fail_archive:assert any('archive_error' in row for row in coordinator.scan_bbox_evidence['last_depth'])
        print(json.dumps(dict(PASS=True,synthetic_only=True,empty=empty,cancel_elapsed_s=canceled,evidence=coordinator.scan_bbox_evidence)))
    finally:
        producer.destroy_timer(timer);producer.destroy_timer(cam_timer);coordinator.destroy_node();server.destroy();runtime.pool.shutdown(wait=False,cancel_futures=True)
        runtime.destroy_node();producer.destroy_node();executor.shutdown();rclpy.shutdown()

if __name__=='__main__':main()
