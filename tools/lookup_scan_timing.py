"""Retime only ScanPose, preserving every checked joint waypoint and metadata."""
from copy import deepcopy
import numpy as np
from builtin_interfaces.msg import Duration
from deux_task_runtime.common import seconds,ros_time
from deux_task_runtime.trajectory import arrays,validate_limits
from deux_task_core.model import JOINT_NAMES


def retime_scan(plan,config,robot,requested_scale):
    if plan.stage!='ScanPose':raise ValueError('ScanPose only')
    original=float(config.get('scan_motion_scale',1.));requested=float(requested_scale)
    if not original<=requested<=1.:raise ValueError('scan speed override must be between configured scale and 1')
    if plan.follows_conveyor or plan.grasp_candidate_id:raise ValueError('stored/interception timing must not be retimed')
    times,positions=arrays(plan.trajectory,JOINT_NAMES);result=deepcopy(plan)
    ratio=original/requested
    for point,t in zip(result.trajectory.points,times):point.time_from_start=ros_time(float(t*ratio),Duration)
    result.phase_end_times=[float(t*ratio) for t in result.phase_end_times]
    validate_limits(result.trajectory,JOINT_NAMES,robot)
    return result,dict(configured_scale=original,requested_scale=requested,
        original_duration_s=float(times[-1]),duration_s=float(times[-1]*ratio),waypoints_unchanged=True)
