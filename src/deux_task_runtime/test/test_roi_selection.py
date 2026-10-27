from types import SimpleNamespace as NS
import pytest
pytest.importorskip('rclpy')
from deux_task_msgs.msg import DetectionArray, BoundingBox, VisualEvidence, VisualEvidenceArray, PerceptionRequest
from deux_task_runtime.common import ros_time
from deux_task_runtime.head_task_node import HeadTaskNode


def fixture():
    tracks=DetectionArray(sequence=3,request_id='r',selection_roi_valid=True,
        selection_roi_xyxy=[100.,100.,500.,400.],boxes=[BoundingBox(
            class_name='bread_bag',track_id='bag',confidence=.3,xyxy=[200.,200.,300.,300.])])
    tracks.header.stamp=ros_time(10.);tracks.header.frame_id='camera'
    c=VisualEvidence(target_id='bag',bag_visible=True,bbox3d_valid=True)
    # Outside the old Z/XYZ selection gate, but a valid depth box in the image ROI.
    c.bbox3d_min.x=.48;c.bbox3d_min.y=.2;c.bbox3d_min.z=.83
    c.bbox3d_max.x=.57;c.bbox3d_max.y=.3;c.bbox3d_max.z=.84
    candidates=VisualEvidenceArray(sequence=3,request_id='r',candidates=[c])
    candidates.header.stamp=ros_time(10.);candidates.header.frame_id='base_link'
    req=PerceptionRequest(request_id='r');req.header.stamp=ros_time(9.9)
    node=NS(tracks=tracks,candidates=candidates,perception_request=req,reason='',target_id='',now=lambda:10.,
        config=dict(target_selection_policy='rgb_roi_3d_bbox',base_frame='base_link',max_object_age=.6,
                    moving_source=True,conveyor_tracking=dict(flow_direction=[0.,-1.,0.])))
    return node,c


def test_one_frame_inside_roi_and_depth_box_is_sufficient():
    node,_=fixture()
    assert HeadTaskNode.select_ready(node,{})
    assert node.target_id=='bag' and not node.reason


@pytest.mark.parametrize('case', ['roi_missing','outside','no_box','nan_box','old_frame','old_request',
                                  'old_image','wrong_frame','requested_other'])
def test_bad_or_unmatched_evidence_never_selects(case):
    node,c=fixture();requested=''
    if case=='roi_missing':node.tracks.selection_roi_valid=False
    if case=='outside':node.tracks.boxes[0].xyxy=[510.,200.,550.,300.]
    if case=='no_box':c.bbox3d_valid=False
    if case=='nan_box':c.bbox3d_min.x=float('nan')
    if case=='old_frame':node.candidates.sequence=2
    if case=='old_request':node.candidates.request_id='old'
    if case=='old_image':node.tracks.header.stamp=ros_time(9.)
    if case=='wrong_frame':node.candidates.header.frame_id='camera'
    if case=='requested_other':requested='other'
    assert not HeadTaskNode.select_ready(node,{},requested)
    assert node.reason and not node.target_id


def test_relaxed_estimate_skips_extra_velocity_window_but_requires_usable_icp():
    from test_estimate_readiness import setup
    node,target=setup();node.config['target_selection_policy']='rgb_roi_3d_bbox'
    target.filter_observations=3
    # Would fail the old predicted completion window; the actual planner still checks it.
    target.pose.pose.position.y=-.15
    assert HeadTaskNode.estimate_ready(node,[])
    target.velocity_valid=False
    assert not HeadTaskNode.estimate_ready(node,[])
    assert '속도 추정' in node.reason


def test_prepare_uses_same_one_frame_rule_without_dispatching_motion():
    node,_=fixture();calls=[]
    node.state='Prepare';node.requested_target_id='';node.perception_request.mode='detect'
    node.request_perception=calls.append
    HeadTaskNode.receive_candidates(node,node.candidates)
    assert node.target_id=='bag' and calls==['locate']


def test_lane_ranking_prefers_crossing_workspace_without_excluding_other_lanes():
    from copy import deepcopy
    node,c=fixture()
    node.config['conveyor_tracking']['acquisition_region']=dict(min=[.45,-.48,.68],max=[.6,.4,.82])
    c.bbox3d_min.z=.70;c.bbox3d_max.z=.76
    other=deepcopy(node.tracks.boxes[0]);other.track_id='adjacent';node.tracks.boxes.append(other)
    adjacent=deepcopy(c);adjacent.target_id='adjacent'
    adjacent.bbox3d_min.x=.62;adjacent.bbox3d_max.x=.70
    adjacent.bbox3d_min.y=.3;adjacent.bbox3d_max.y=.4
    node.candidates.candidates.append(adjacent)
    assert HeadTaskNode.select_ready(node,{}) and node.target_id=='bag'
    # A valid requested candidate remains eligible even in the other lane.
    assert HeadTaskNode.select_ready(node,{},'adjacent') and node.target_id=='adjacent'
