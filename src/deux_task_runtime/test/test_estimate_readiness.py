import asyncio
from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
from geometry_msgs.msg import Pose, Twist
from deux_task_runtime.head_task_node import HeadTaskNode
from deux_task_runtime.common import ros_time
from deux_task_runtime.behavior_tree import Failure


def setup():
    pose=Pose();pose.orientation.w=1.;pose.position.x=.52;pose.position.z=.75
    velocity=Twist();velocity.linear.y=-.08
    target=NS(pose_filtered=True,velocity_valid=True,filter_observations=6,velocity=velocity,pose=NS(pose=pose))
    cfg=dict(min_motion_observations=6,velocity_stability_m_s=.004,start_lead=.4,intercept_wait_s=1.,
             durations=dict(Approach=1.5,Track=0.,Descend=.8,Close=1.),boundary_margin=.015,
             acquisition_region=dict(min=[.45,-.48,.68],max=[.6,.4,.82]),
             grasp_region=dict(min=[.45,-.48,.68],max=[.6,.12,.82]))
    node=NS(config=dict(moving_source=True,conveyor_tracking=cfg),target_id='bag',reason='',now=lambda:10.,
            target=lambda _:target,perception_request=NS(request_id='request'),
            observation=NS(objects=NS(request_id='request',sequence=1,header=NS(stamp=ros_time(10.)))))
    return node,target


def warm(node):
    samples=[]
    for i in range(3):
        node.observation.objects.sequence=i
        result=HeadTaskNode.estimate_ready(node,samples)
    return result


@pytest.mark.parametrize('case,reason',[
    ('request','ICP 요청'),('filtered','칼만 필터'),('velocity','속도 추정'),
    ('count','2/6'),('stale','오래됨'),('missing','선택 ID'),('exit','파지 완료 예측'),
    ('arrival','접근 도착 예측')])
def test_blocking_reason_is_exposed(case,reason):
    node,target=setup()
    if case=='request':node.observation.objects.request_id='old'
    if case=='filtered':target.pose_filtered=False
    if case=='velocity':target.velocity_valid=False
    if case=='count':target.filter_observations=2
    if case in ('stale','missing'):
        def reject(_):raise ValueError('object stale/future' if case=='stale' else 'required object pose unavailable')
        node.target=reject
    if case=='exit':target.pose.pose.position.y=-.15
    if case=='arrival':target.pose.pose.position.y=.4
    assert not warm(node) and reason in node.reason


def test_velocity_instability_and_success_clear_reason():
    node,target=setup();samples=[]
    for i,speed in enumerate([-.08,-.09,-.08]):
        target.velocity.linear.y=speed;node.observation.objects.sequence=i
        assert not HeadTaskNode.estimate_ready(node,samples)
    assert '속도 불안정' in node.reason
    assert warm(node) and node.reason==''


def test_repeated_sample_does_not_count_as_new_velocity_evidence():
    node,_=setup();samples=[]
    for _ in range(5):assert not HeadTaskNode.estimate_ready(node,samples)
    assert '1/3' in node.reason


def test_timeout_preserves_last_blocking_reason():
    node,_=setup();node.state='EstimatePose';node.reason='칼만 관측 횟수 부족: 2/6회'
    with pytest.raises(Failure,match='2/6회'):
        asyncio.run(HeadTaskNode.await_condition(node,lambda:False,None,None,timeout=0.))
