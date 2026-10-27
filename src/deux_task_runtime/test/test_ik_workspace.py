from copy import deepcopy
from types import SimpleNamespace as NS
import numpy as np
import pytest
pytest.importorskip('rclpy')
from deux_task_msgs.msg import MotionPlan,StateObservation,ObjectDetection
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_runtime.common import ros_time
from deux_task_runtime.execution_node import ExecutionNode
from deux_task_core.conveyor import validate_occlusion_prediction


def fixture():
    obj=ObjectDetection(track_id='bag',pose_filtered=True,velocity_valid=True,motion_covariance_valid=True)
    obj.pose.pose.orientation.w=1.;obj.pose.pose.position.x=.65;obj.pose.pose.position.y=.15;obj.pose.pose.position.z=.75
    obj.motion_covariance=(np.eye(8)*1e-8).ravel().tolist()
    obs=StateObservation(objects_valid=True);obs.objects.header.stamp=ros_time(10.);obs.objects.detections=[obj]
    plan=MotionPlan(stage='Approach',target_id='bag',follows_conveyor=True,target_acquired_at=ros_time(9.9))
    plan.target_pose=deepcopy(obj.pose.pose);plan.trajectory.header.stamp=ros_time(10.2)
    plan.trajectory.points=[JointTrajectoryPoint(time_from_start=Duration(sec=1))]
    region=dict(min=[.45,-.48,.68],max=[.6,.12,.82])
    config=dict(moving_source=True,max_object_age=.6,max_target_drift=.025,
        conveyor_tracking=dict(acquisition_region=region,grasp_region=region,boundary_margin=.015,
            max_plan_horizon=8.,occlusion_prediction_max_age=3.,prediction_acceleration_std=.001))
    node=NS(config=config,observation=obs,now=lambda:10.,target=lambda *a,**kw:obj)
    return node,obj,plan


def test_execution_does_not_reintroduce_box_but_keeps_innovation_and_expiry():
    node,obj,plan=fixture()
    with pytest.raises(ValueError,match='region'):ExecutionNode.conveyor_guard(node,plan)
    node.config['conveyor_tracking']['workspace_policy']='ik'
    ExecutionNode.conveyor_guard(node,plan)
    plan.stage='TrackGrasp'
    ExecutionNode.conveyor_guard(node,plan)
    obj.pose.pose.position.x+=.1
    with pytest.raises(ValueError,match='innovation'):ExecutionNode.conveyor_guard(node,plan)
    obj.pose.pose.position.x-=.1;plan.target_acquired_at=ros_time(1.)
    with pytest.raises(ValueError,match='expired'):ExecutionNode.conveyor_guard(node,plan)


def test_occluded_ik_workspace_keeps_covariance_limit_without_box():
    node,obj,plan=fixture()
    with pytest.raises(ValueError,match='region'):
        validate_occlusion_prediction(obj,plan,.7,10.,node.config)
    node.config['conveyor_tracking']['workspace_policy']='ik'
    assert validate_occlusion_prediction(obj,plan,.7,10.,node.config)<.025
    obj.motion_covariance=(np.eye(8)*.1).ravel().tolist()
    with pytest.raises(ValueError,match='uncertainty'):
        validate_occlusion_prediction(obj,plan,.7,10.,node.config)
