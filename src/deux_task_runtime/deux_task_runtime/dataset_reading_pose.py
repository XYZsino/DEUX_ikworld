"""Named, measured arm posture for a fixed barcode presentation; no camera mapping."""
import json
from pathlib import Path
import numpy as np
from deux_task_core.model import RobotModel
from deux_task_core.planning import hermite


def load_pose(filename):
    if not filename:return None
    data=json.loads(Path(filename).read_text())
    if data.get('schema')!='deux.dataset_reading_pose.v1':raise ValueError('unknown dataset reading pose schema')
    if data.get('applied_arms')!='left_only':raise ValueError('dataset posture must apply left arm only')
    for side in ['left']:
        joints=np.asarray(data[side+'_joints_rad'],float)
        if joints.shape!=(7,) or not np.isfinite(joints).all():raise ValueError('seven finite arm joint angles required')
    return data


def validate_endpoint(pose,positions):
    end=np.asarray(positions,float)[-1]
    if end.shape!=(28,):raise ValueError('28-joint reading path required')
    if not np.allclose(end[:7],pose['left_joints_rad'],atol=1e-7,rtol=0.):raise ValueError('dataset reading endpoint mismatch: left')
    path=np.asarray(positions,float)
    if not np.allclose(path[:,14:21],path[0,14:21],atol=1e-10,rtol=0.):raise ValueError('existing right arm must remain stationary during reading alignment')


def build_reading_plan(config,pose,actual):
    robot=RobotModel(config['robot']);start=np.asarray(actual,float).copy()
    if start.shape!=(28,) or not np.isfinite(start).all():raise ValueError('finite measured start required')
    if config.get('hand_command_only',False):
        start[7:14]=config['hand_closed'][:7];start[21:28]=config['hand_closed'][7:]
    goal=start.copy();goal[:7]=pose['left_joints_rad']
    for q in [start,goal]:
        if np.any(q<robot.limits[:,0]-robot.joint_limit_tolerance) or np.any(q>robot.limits[:,1]+robot.joint_limit_tolerance):raise ValueError('reading posture joint limit')
    scale=float(config.get('scan_motion_scale',.25));t,path=hermite([start,goal],robot.velocity*scale,robot.acceleration*scale**2,config['sample_period'])
    validate_endpoint(pose,path)
    return dict(times=t,path=path,minimum_clearance=0.,ik_calls=0,
        dataset_reading_reference=dict(episode=pose['episode'],frame=pose['frame'],timestamp_s=pose['timestamp_s']))
