from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from deux_task_runtime.head_task_node import HeadTaskNode
from deux_task_runtime.common import ros_time
from deux_task_msgs.msg import VisualEvidence,VisualEvidenceArray,PerceptionRequest


def fixture():
    calls=[]
    node=SimpleNamespace(state='Prepare',target_id='',requested_target_id='',
        config=dict(base_frame='base_link',moving_source=True,max_object_age=.6,conveyor_tracking=dict(
            grasp_region=dict(min=[.45,-.48,.68],max=[.6,.12,.82]),boundary_margin=.015,
            acquisition_region=dict(min=[.45,-.48,.68],max=[.6,.4,.82]),flow_direction=[0,-1,0])),
        perception_request=PerceptionRequest(mode='detect',request_id='session'),
        track_history={'near':[9.5,9.6,9.7],'upstream':[9.5,9.6,9.7]},
        now=lambda:10.,request_perception=calls.append)
    msg=VisualEvidenceArray(request_id='session');msg.header.stamp=ros_time(10.);msg.header.frame_id='base_link'
    for name,y in [('near',.2),('upstream',.3)]:
        item=VisualEvidence(target_id=name,bag_visible=True,surface_center_valid=True)
        item.surface_center.x=.52;item.surface_center.y=y;item.surface_center.z=.73;msg.candidates.append(item)
    return node,msg,calls


def test_prepare_only_preselects_and_requests_perception():
    node,msg,calls=fixture();HeadTaskNode.receive_candidates(node,msg)
    assert node.target_id=='upstream' and calls==['locate']
    # There is deliberately no planner/executor on this object: background
    # preparation tracking must never dispatch a robot command.
    HeadTaskNode.receive_candidates(node,msg)
    assert calls==['locate']


@pytest.mark.parametrize('case',['idle','stale','other_request','unstable','outside','wrong_frame','already_in_grasp'])
def test_prepare_tracking_rejects_ineligible_input(case):
    node,msg,calls=fixture()
    if case=='idle':node.state='Idle'
    if case=='stale':msg.header.stamp=ros_time(9.)
    if case=='other_request':msg.request_id='other'
    if case=='wrong_frame':msg.header.frame_id='camera'
    if case=='unstable':node.track_history={}
    if case=='already_in_grasp':
        for c in msg.candidates:c.surface_center.y=0.
    if case=='outside':
        for c in msg.candidates:c.surface_center.x=1.
    HeadTaskNode.receive_candidates(node,msg)
    assert node.target_id=='' and not calls


def test_requested_identity_is_respected():
    node,msg,calls=fixture();node.requested_target_id='near'
    HeadTaskNode.receive_candidates(node,msg)
    assert node.target_id=='near' and calls==['locate']


@pytest.mark.parametrize('stage,reset',[('Prepare',True),('Approach',False)])
def test_track_loss_reselects_only_before_approach(stage,reset):
    from deux_task_msgs.msg import DetectionArray
    node,msg,calls=fixture();node.state=stage;node.target_id='upstream'
    node.prepare_target_last_seen=9.;node.tracks=None
    tracks=DetectionArray();tracks.header.stamp=ros_time(10.)
    HeadTaskNode.receive_tracks(node,tracks)
    assert (node.target_id=='')==reset
    assert calls==(['detect'] if reset else [])


def test_rendezvous_uses_measured_ee_and_predicted_arrival_without_track_dwell():
    import numpy as np
    from deux_task_msgs.msg import StateObservation,ObjectDetection
    obs=StateObservation(objects_valid=True,right_ee_valid=True)
    obs.objects.header.stamp=ros_time(9.9)
    obs.base_from_right_ee.header.stamp=ros_time(10.)
    obj=ObjectDetection(track_id='bag',pose_filtered=True,velocity_valid=True)
    obj.pose.pose.position.x=.5;obj.pose.pose.position.y=0.;obj.pose.pose.position.z=.7
    obj.pose.pose.orientation.w=1.;obj.velocity.linear.y=-.08
    obs.objects.detections=[obj]
    ee=obs.base_from_right_ee.transform
    ee.translation.x=.5;ee.translation.y=-.04;ee.translation.z=.77;ee.rotation.w=1.
    cfg=dict(max_object_age=.6,max_observation_age=.3,approach_height=.07,
        grasp_object_from_right_ee=np.eye(4).tolist(),conveyor_tracking=dict(
        occlusion_prediction_max_age=3.,start_lead=.4,durations=dict(Track=0.),
        grasp_region=dict(min=[.4,-.5,.6],max=[.6,.2,.8]),boundary_margin=.01,
        intercept_trigger_distance_m=.025,intercept_height_tolerance_m=.02))
    node=SimpleNamespace(config=cfg,target_id='bag',now=lambda:10.,
        planning_observation=lambda stage:obs,target=lambda *args,**kwargs:obj)
    assert HeadTaskNode.rendezvous_ready(node)
    ee.translation.y=.1
    assert not HeadTaskNode.rendezvous_ready(node)
    ee.translation.y=-.04;ee.translation.z=.9
    assert not HeadTaskNode.rendezvous_ready(node)


def test_cached_planning_pose_keeps_acquisition_and_live_joints():
    from copy import deepcopy
    from deux_task_msgs.msg import StateObservation, ObjectDetection
    from deux_task_runtime.common import RuntimeNode
    node,_,_=fixture()
    node.config['model_id']='model'
    node.config['conveyor_tracking'].update(occluded_grasp_enabled=True,occlusion_prediction_max_age=3.)
    node.target_id='bag';node.observation=StateObservation()
    node.observation.joints_acquired_at=ros_time(10.)
    node.observation.objects_valid=False
    node.grasp_objects=deepcopy(node.observation.objects)
    node.grasp_objects.header.stamp=ros_time(8.)
    node.grasp_objects.header.frame_id='base_link';node.grasp_objects.model_id='model'
    node.grasp_objects.detections=[ObjectDetection(track_id='bag')]
    node.target=lambda identity,obs=None,max_age=None:RuntimeNode.target(node,identity,obs=obs,max_age=max_age)
    result=HeadTaskNode.planning_observation(node,'TrackGrasp')
    assert result.objects.header.stamp==ros_time(8.)
    assert result.joints_acquired_at==ros_time(10.)
    assert not node.observation.objects_valid
    node.grasp_objects.header.stamp=ros_time(6.)
    with pytest.raises(ValueError,match='stale'):HeadTaskNode.planning_observation(node,'TrackGrasp')
