"""LeaderGrip integration over isolated ROS domains; never production hardware."""
import time
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest
import yaml
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.action import ActionClient,ActionServer,GoalResponse,CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.srv import GetParameters
from control_msgs.action import FollowJointTrajectory
from deux_hand_msgs.msg import LeaderGrip,HandCtlMessage,MotorData,HandUiMessage
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.msg import MotionPlan
from deux_task_msgs.action import ExecuteMotion
from deux_task_core.model import JOINT_NAMES,HAND_NAMES,ARM_NAMES
from deux_task_core.synthetic import fixture
from deux_task_core.planning import fixed_hand_path,scene_version
from deux_task_runtime.execution_node import ExecutionNode
from deux_task_runtime.common import ros_time,seconds,pause
from deux_task_runtime.leader_grip_adapter import grip_fraction,LeaderGripHandAdapter
from test_nodes import Harness,params

def mapping():
    d=dict(protocol='leader_grip',feedback_mapping_confirmed=True,keep_left_closed=True)
    for side in ('left','right'):
        names=[n for n in HAND_NAMES if n.startswith(side)]
        d[side]=dict(feedback_topic=f'/hand/{side}/info',feedback_type='deux_hand_msgs/msg/HandCtlMessage',
          leader_mode=3,initial_rad=[0.]*7,target_rad=[.03]*7,
          joints={n:dict(id=i+1,feedback_name=str(i+1),sign=1,offset=0.,min=-3.,max=3.,velocity=7.)
                  for i,n in enumerate(names)})
    return d

def test_scalar_projection_rejects_arbitrary_finger_postures():
    a=np.zeros(7);b=np.arange(1,8)*.1
    assert grip_fraction(a+.4*b,a,b,1e-6)==pytest.approx(.4)
    q=a+.4*b;q[0]+=.2
    with pytest.raises(ValueError,match='outside'):grip_fraction(q,a,b,.05)
    with pytest.raises(ValueError):grip_fraction(a,a,a,.05)

@pytest.mark.parametrize('case',['continuous'])
def test_execution_dispatches_scalar_grip_and_keeps_reader_closed(tmp_path,case):
    c=fixture(tmp_path);c['hand_open'][:7]=c['hand_closed'][:7]
    c['hand_command_only']=True
    c['placement']={'enabled':False,'continuous_motion':True,'fixed_joint_motion':True}
    c["max_joint_age"]=.1
    c['hand_close_duration_s']=c['hand_open_duration_s']=.5
    if case.startswith('preload'):
        c['right_hand_preload']=True;c['start_tolerance']=.005;c['tracking_tolerance']=.01
    if case=='binary_stall':c['hand_command_only']=True;c['start_tolerance']=.005
    if case.startswith('review_'):
        c['motion_review_required']=True;c['motion_review_timeout_s']=1. if case=='review_timeout' else 5.
    if case.startswith('late_'):c['settle_timeout']=.05
    h=Harness(c);h.source.group=ReentrantCallbackGroup()
    h.q[7:14]=.03
    h.q[21:28]=.03
    path=tmp_path/'mapping.yaml';path.write_text(yaml.safe_dump(mapping()))
    node=h.add(ExecutionNode(parameter_overrides=params(config_file=str(h.config_path),
        mapping_file=str(path),hardware_enabled=True,dry_run=False,driver_action='/synthetic/arm')))
    assert isinstance(node.hand,LeaderGripHandAdapter)
    emitted=[];canceled=[];enabled=[True];mode=[3];opening=[False]
    def grip(side,msg):
        assert msg.valid and msg.side==side
        emitted.append((side,msg.grasp))
        for name in [n for n in HAND_NAMES if n.startswith(side)]:
            value=.03*msg.grasp
            if side=='right' and case.startswith('preload'):
                value=min(value,.01)
                if case=='preload_open_stall' and opening[0]:value=max(value,.01)
            if case=='binary_stall':value=.01
            h.q[JOINT_NAMES.index(name)]=value
    subs=[h.source.create_subscription(LeaderGrip,f'/hand/{s}/grip',
          lambda msg,s=s:grip(s,msg),qos_profile_sensor_data) for s in ('left','right')]
    fps={s:h.source.create_publisher(HandCtlMessage,f'/hand/{s}/info',qos_profile_sensor_data) for s in ('left','right')}
    ups={s:h.source.create_publisher(HandUiMessage,f'/hand/{s}/ui_response',10) for s in ('left','right')}
    def feedback():
        for side in ('left','right'):
            ups[side].publish(HandUiMessage(run_flag=True,stop_flag=False,teleop_mode=0,leader_mode=mode[0]))
            if not enabled[0]:continue
            msg=HandCtlMessage(side=side);msg.header.stamp=h.source.get_clock().now().to_msg()
            msg.motors=[MotorData(id=i+1,torque=True,rad=float(h.q[JOINT_NAMES.index(n)]),
                                 hardware_error_status=0)
                        for i,n in enumerate([n for n in HAND_NAMES if n.startswith(side)])]
            fps[side].publish(msg)
    timer=h.source.create_timer(.02,feedback)
    turn=[0]
    def get_parameters(req,response):
        side=('left','right')[turn[0]%2];turn[0]+=1
        values={'hand_side':side,'leader_mode_3.initial_deg':[0.]*7,
                'leader_mode_3.target_deg':np.rad2deg([.03]*7).tolist()}
        response.values=[Parameter(k,value=values[k]).get_parameter_value() for k in req.names]
        return response
    service=h.source.create_service(GetParameters,'/deux_hand/get_parameters',get_parameters)
    arm_requests=[]
    async def arm(goal):
        arm_requests.append(goal.request)
        end=seconds(goal.request.trajectory.header.stamp)+seconds(goal.request.trajectory.points[-1].time_from_start)
        if case=='late_arm':end+=.6
        if case=='late_cancel':end+=5.
        while h.source.get_clock().now().nanoseconds*1e-9<end:
            if goal.is_cancel_requested:
                canceled.append(True);goal.canceled()
                return FollowJointTrajectory.Result(error_code=-1)
            await pause(h.source)
        goal.succeed();return FollowJointTrajectory.Result(error_code=0)
    server=ActionServer(h.source,FollowJointTrajectory,'/synthetic/arm',execute_callback=arm,
         goal_callback=lambda _:GoalResponse.ACCEPT,cancel_callback=lambda _:CancelResponse.ACCEPT,
         callback_group=h.source.group)
    client=ActionClient(h.source,ExecuteMotion,'/task/execute_motion')
    try:
        h.spin(lambda: len(node.hand.raw)==2 and len(node.hand.profile_state)==2 and
               len(node.hand.mode_state)==2 and node.arm.state is not None and
               node.observation is not None and client.server_is_ready() and node.arm.client.server_is_ready())
        def dispatch(stage,identity):
            start=h.q.copy();goal=start.copy();goal[21:28]=.03 if stage=='Close' else 0.
            t,q=fixed_hand_path([start,goal],np.ones(28)*1.5,np.ones(28)*4,.02,.5)
            p=MotionPlan(plan_id=identity,scene_version=scene_version(c),target_id='bag',
                         stage=stage,model_id=c['model_id'],synthetic=True)
            p.stage='PlaceApproach'
            t=np.arange(76)*.02;q=np.tile(start,(76,1))
            u=np.clip((t-.5)/.5,0,1);q[:,21:28]=.03*(1-(3*u*u-2*u*u*u))[:,None]
            p.phase_end_times=[.5,1.,1.5]
            p.header.stamp=h.source.get_clock().now().to_msg();p.header.frame_id='base_link'
            p.valid_until=ros_time(seconds(p.header.stamp)+3)
            p.target_pose=deepcopy(h.observation.objects.detections[0].pose.pose)
            p.trajectory.joint_names=list(JOINT_NAMES)
            p.trajectory.points=[JointTrajectoryPoint(positions=row.tolist(),time_from_start=ros_time(float(ti),Duration)) for ti,row in zip(t,q)]
            handle=h.future(client.send_goal_async(ExecuteMotion.Goal(plan=p)))
            assert handle.accepted
            return handle
        handle=dispatch('Close','close')
        if case.startswith('review_'):
            from std_msgs.msg import String
            import json
            h.spin(lambda:node.review.ticket is not None)
            for _ in range(4):h.ex.spin_once(timeout_sec=.02)
            assert not emitted and not arm_requests
            ticket=node.review.ticket
            payload=dict(request_id=ticket.request_id,plan_id=ticket.plan_id,digest=ticket.digest,decision='approve')
            node.review.receive(String(data=json.dumps(dict(payload,request_id='old'))))
            assert node.review.ticket.decision is None and not emitted
            if case=='review_cancel':h.future(handle.cancel_goal_async())
            elif case!='review_timeout':
                if case=='review_moved':
                    h.q[0]=.5
                    # Ensure fresh arm samples reflect motion before approval.
                    for _ in range(12):h.ex.spin_once(timeout_sec=.02)
                if case=='review_reject':payload['decision']='reject'
                node.review.receive(String(data=json.dumps(payload)))
            result=h.future(handle.get_result_async(),10).result
            assert result.success==(case=='review_accept'),result.reason
            if case!='review_accept':assert not emitted
            assert len(arm_requests)==(1 if case=="review_accept" else 0)
            assert node.review.ticket is None
            return
        h.spin(lambda:any(s=='right' and f>.05 for s,f in emitted))
        if case=='late_cancel':
            until=time.monotonic()+.8
            while time.monotonic()<until:h.ex.spin_once(timeout_sec=.02)
            assert not handle.get_result_async().done()
            h.future(handle.cancel_goal_async())
        elif case=='cancel':h.future(handle.cancel_goal_async())
        elif case=='feedback_loss':enabled[0]=False
        elif case=='mode_loss':mode[0]=2
        elif case=='profile_mismatch':
            node.hand.profile_timer.cancel()
            node.hand.profile_state['right']=(time.monotonic(),False)
        result=h.future(handle.get_result_async(),10).result
        if case=='continuous':
            assert result.success,result.reason
            assert len(arm_requests)==1
            right=[v for side,v in emitted if side=='right']
            assert right[0]==1. and right[-1]==0.
            first_open=right.index(0.)
            assert all(v==1. for v in right[:first_open]) and all(v==0. for v in right[first_open:])
            return
        if case.startswith('late_'):
            assert result.success==(case=='late_arm'),result.reason
            return
        assert result.success==(case in ('complete','binary_stall') or case.startswith('preload')),result.reason
        if case=='binary_stall':assert all(f in (0.,1.) for _,f in emitted)
        assert emitted and all(abs(f-1.)<1e-6 for s,f in emitted if s=='left')
        assert not h.source.get_publishers_info_by_topic('/hand/right/cmd')
        if case in ('complete','binary_stall') or case.startswith('preload'):
            opening[0]=True
            before=len(emitted)
            handle=dispatch('Release','release')
            if case=='preload_open_stall':
                future=handle.get_result_async();until=time.monotonic()+3.
                while time.monotonic()<until:h.ex.spin_once(timeout_sec=.02)
                assert not future.done()  # No completion deadline; still waiting for open feedback.
                h.future(handle.cancel_goal_async())
                result=h.future(future,5).result
            else:
                result=h.future(handle.get_result_async(),10).result
            assert result.success==(case!='preload_open_stall'),result.reason
            assert emitted[-1][0]=='right' and emitted[-1][1]==pytest.approx(0.)
            assert all(f==pytest.approx(1.) for s,f in emitted[before:] if s=='left')
        else:
            count=len(emitted)
            # A canceled/stopped scalar stream must not synthesize an opening.
            for _ in range(5):h.ex.spin_once(timeout_sec=.02)
            assert len(emitted)==count
    finally:
        h.close()


def test_preload_masks_only_right_close_and_hold_not_other_feedback():
    from types import SimpleNamespace
    node=SimpleNamespace(config=dict(right_hand_preload=True,hand_closed=[.1]*14),
                         hand=SimpleNamespace(mapping=dict(protocol='leader_grip')))
    measured=np.zeros(28);expected=np.zeros(28);expected[21:28]=.1
    close=SimpleNamespace(stage='Close')
    assert ExecutionNode.position_error(node,measured,expected,close,'final')==0
    assert ExecutionNode.position_error(node,measured,expected,close,'start')==pytest.approx(.1)
    expected[0]=.2
    assert ExecutionNode.position_error(node,measured,expected,close,'final')==pytest.approx(.2)
    expected[0]=0;expected[7]=.3
    assert ExecutionNode.position_error(node,measured,expected,close,'final')==pytest.approx(.3)
    expected[7]=0
    assert ExecutionNode.position_error(node,measured,expected,SimpleNamespace(stage='Release'),'final')==pytest.approx(.1)


def test_command_only_masks_both_hands_but_never_arms():
    node=SimpleNamespace(config=dict(hand_command_only=True),
                         hand=SimpleNamespace(mapping=dict(protocol='leader_grip')))
    measured=np.zeros(28);expected=np.zeros(28)
    expected[7:14]=1.;expected[21:28]=2.
    for stage in ('Prepare','Close','Release','Lift','ReadyPose','PlaceRetreat'):
        for phase in ('start','tracking','final'):
            assert ExecutionNode.position_error(node,measured,expected,SimpleNamespace(stage=stage),phase)==0.
    expected[14]=.2
    assert ExecutionNode.position_error(node,measured,expected,SimpleNamespace(stage='Close'),'final')==pytest.approx(.2)
