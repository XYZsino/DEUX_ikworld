"""Selection diagnostics must preserve target choice and prevent false success."""
import asyncio
from copy import deepcopy
from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
from deux_task_runtime.head_task_node import HeadTaskNode
from deux_task_runtime.common import ros_time
from deux_task_runtime.behavior_tree import Failure


def setup():
    box=NS(class_name='bread_bag',confidence=.95,track_id='bag',xyxy=[440,200,520,280])
    candidate=NS(target_id='bag',bag_visible=True,surface_center_valid=True,surface_center=NS(x=.52,y=.2,z=.75))
    node=NS(reason='',target_id='',now=lambda:10.,perception_request=NS(request_id='request'),
        config=dict(moving_source=True,max_object_age=.6,conveyor_tracking=dict(flow_direction=[0,-1,0],
            acquisition_region=dict(min=[.45,-.48,.68],max=[.6,.4,.82]))),
        tracks=NS(header=NS(stamp=ros_time(10.)),boxes=[box]),
        candidates=NS(request_id='request',header=NS(stamp=ros_time(10.)),candidates=[candidate]))
    return node,box,candidate


@pytest.mark.parametrize('case,reason',[
    ('none','수신 대기'),('stale','데이터 시각 불충족'),('future','데이터 시각 불충족'),
    ('no_bag','bread_bag 0개'),('confidence','신뢰도 부족'),('no_id','ID 발급 대기'),
    ('requested','지정 대상 ID 미검출'),('no_candidates','3D 후보 수신 대기'),
    ('old_request','요청에 대응'),('old_candidates','3D 후보 데이터 시각 불충족'),
    ('missing_id','같은 ID의 3D 후보 없음'),('invisible','가시성 부족'),
    ('no_depth','유효한 깊이 좌표 없음'),('nan_depth','유효한 깊이 좌표 없음'),('outside','선택 영역 밖')])
def test_selection_reports_each_unmet_gate(case,reason):
    node,box,candidate=setup();requested=''
    if case=='none':node.tracks=None
    if case=='stale':node.tracks.header.stamp=ros_time(9.)
    if case=='future':node.tracks.header.stamp=ros_time(11.)
    if case=='no_bag':box.class_name='barcode'
    if case=='confidence':box.confidence=.4
    if case=='no_id':box.track_id=''
    if case=='requested':requested='other'
    if case=='no_candidates':node.candidates=None
    if case=='old_request':node.candidates.request_id='old'
    if case=='old_candidates':node.candidates.header.stamp=ros_time(9.)
    if case=='missing_id':candidate.target_id='other'
    if case=='invisible':candidate.bag_visible=False
    if case=='no_depth':candidate.surface_center_valid=False
    if case=='nan_depth':candidate.surface_center.x=float('nan')
    if case=='outside':candidate.surface_center.y=-.6
    assert not HeadTaskNode.select_ready(node,{},requested)
    assert reason in node.reason and node.target_id==''
    if case=='outside':assert '-60.0' in node.reason and '-48.0~40.0' in node.reason


def test_observations_need_distinct_frames_and_success_clears_reason():
    node,_,_=setup();observations={}
    for _ in range(4):assert not HeadTaskNode.select_ready(node,observations)
    assert '1/3회' in node.reason
    for stamp in [10.1,10.2]:
        node.now=lambda:stamp
        node.tracks.header.stamp=node.candidates.header.stamp=ros_time(stamp)
        accepted=HeadTaskNode.select_ready(node,observations)
    assert accepted and node.target_id=='bag' and not node.reason


def test_upstream_ranking_and_explicit_identity_are_preserved():
    node,box,candidate=setup();other=deepcopy(box);other.track_id='upstream'
    c=deepcopy(candidate);c.target_id=other.track_id;c.surface_center.y=.3
    node.tracks.boxes.append(other);node.candidates.candidates.append(c)
    seen={'bag':{9.8,9.9},'upstream':{9.8,9.9}}
    assert HeadTaskNode.select_ready(node,seen) and node.target_id=='upstream'
    assert HeadTaskNode.select_ready(node,seen,'bag') and node.target_id=='bag'
    # An ineligible candidate must not prevent selection of an eligible one.
    c.surface_center.y=.8
    assert HeadTaskNode.select_ready(node,seen) and node.target_id=='bag'


def test_timeout_retains_mixed_candidate_rejection_counts():
    node,box,candidate=setup();candidate.surface_center.y=-.6
    other=deepcopy(box);other.track_id='missing_depth';node.tracks.boxes.append(other)
    c=deepcopy(candidate);c.target_id=other.track_id;c.surface_center_valid=False
    node.candidates.candidates.append(c)
    assert not HeadTaskNode.select_ready(node,{})
    assert '선택 영역 밖 1개' in node.reason and '유효한 깊이 좌표 없음 1개' in node.reason
    node.state='TrackAndSelect'
    with pytest.raises(Failure,match='선택 영역 밖 1개'):
        asyncio.run(HeadTaskNode.await_condition(node,lambda:False,None,None,timeout=0.))


def test_stationary_selection_does_not_require_depth_candidates():
    node,_,_=setup();node.config['moving_source']=False;node.candidates=None
    assert HeadTaskNode.select_ready(node,{'bag':{9.8,9.9}})
