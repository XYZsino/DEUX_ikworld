from types import SimpleNamespace as NS
from concurrent.futures import Future
import numpy as np
import pytest
pytest.importorskip('rclpy')
from deux_task_msgs.msg import MotionPlan,StateObservation,ObjectDetection
from deux_task_msgs.action import PlanMotion
from trajectory_msgs.msg import JointTrajectoryPoint
from deux_task_runtime.common import ros_time
from deux_task_runtime.grasp_prefetch import GraspPrefetch


def fixture():
    obs=StateObservation(objects_valid=True);obs.objects.header.stamp=ros_time(10.)
    obj=ObjectDetection(track_id='bag',pose_filtered=True,velocity_valid=True)
    obj.pose.pose.orientation.w=1.;obj.velocity.linear.y=-.09
    obs.objects.detections=[obj]
    config=dict(start_tolerance=.05,max_target_drift=.03,max_target_angle_rad=.15,
        conveyor_tracking=dict(max_plan_horizon=8.,max_speed=.15,occluded_grasp_enabled=False,
            candidate_search=dict(time_search=dict(prefetch_grasp=True,max_prediction_std_m=.04))))
    requests=[]
    def submit(*args):
        requests.append(args);f=Future();f.set_result({});return f
    node=NS(config=config,config_version='scene',now=lambda:10.,observation=obs,
        target=lambda identity,o,**kw:o.objects.detections[0],grasp_pose_age=lambda stage:1.,
        pool=NS(submit=submit),get_logger=lambda:NS(info=lambda *args:None))
    plan=MotionPlan(scene_version='scene',target_id='bag',grasp_candidate_id='nominal',plan_id='approach')
    plan.trajectory.header.stamp=ros_time(11.);plan.grasp_start_at=ros_time(13.35)
    plan.grasp_phase_durations=[0.,.8,.9]
    plan.trajectory.points=[JointTrajectoryPoint(positions=[0.]*28)]
    req=PlanMotion.Goal(stage='TrackGrasp',scene_version='scene',target_id='bag',grasp_candidate_id='nominal',
        grasp_start_at=plan.grasp_start_at,grasp_phase_durations=plan.grasp_phase_durations,observation=obs)
    gate=GraspPrefetch(node);gate.arm(plan)
    return gate,node,req,requests


def test_prefetch_uses_predicted_endpoint_during_motion_and_consumes_once():
    gate,node,req,requests=fixture();gate.poll();assert not requests
    node.now=lambda:11.;gate.poll();assert len(requests)==1
    node.now=lambda:13.;c=gate.take(req,np.zeros(28));gate.guard(c,req,np.zeros(28))
    assert requests[0][3]=='TrackGrasp'
    np.testing.assert_array_equal(requests[0][4],np.zeros(28))
    with pytest.raises(ValueError,match='unavailable'):gate.take(req,np.zeros(28))


@pytest.mark.parametrize('change',['target','phase','endpoint','deadline','innovation','scene'])
def test_prefetch_rejects_changed_request_or_execution_evidence(change):
    gate,node,req,_=fixture();node.now=lambda:11.;gate.poll();node.now=lambda:13.
    if change=='target':req.target_id='other'
    if change=='phase':req.grasp_phase_durations=[0.,.8,1.]
    start=np.ones(28) if change=='endpoint' else np.zeros(28)
    if change in ('target','phase','endpoint'):
        with pytest.raises(ValueError):gate.take(req,start)
        return
    c=gate.take(req,start)
    if change=='deadline':node.now=lambda:13.3
    if change=='scene':node.config_version='other'
    if change=='innovation':node.observation.objects.detections[0].pose.pose.position.x=.1
    with pytest.raises(ValueError):gate.guard(c,req,start)
