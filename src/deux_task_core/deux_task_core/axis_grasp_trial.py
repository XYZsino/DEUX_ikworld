"""Opt-in stationary grasp orientation trial; no command publication."""
from copy import deepcopy
from types import SimpleNamespace
import json
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .model import pose, vector


def z_directions(object_rotation, grasp_rotation):
    """Rotate about OBJECT Z before the grasp transform; no position orbit."""
    return [(yaw, object_rotation @ Rotation.from_euler('z', yaw, degrees=True).as_matrix() @ grasp_rotation)
            for yaw in (0, 180)]


def select(planner,start,object_pose,canceled):
    from .planning import Planner
    c=planner.config;r=planner.robot
    if c.get('moving_source',False) or 'descend_distance_m' not in c:
        raise ValueError('axis grasp trial requires stationary fixed-distance descent')
    begin=time.perf_counter();q=vector(start,28).copy();obj=pose(object_pose)
    transform=pose(c['grasp_object_from_right_ee'])
    reference=r.ee_fk('right',np.array(c['home_joints']))[:3,:3]
    choices=[(Rotation.from_matrix(reference.T@rot).magnitude(),yaw,rot)
             for yaw,rot in z_directions(obj[:3,:3],transform[:3,:3])]
    choices.sort(key=lambda x:(x[0],x[1]))
    reports=[]
    for rank,(angle,yaw,rotation) in enumerate(choices):
        if canceled():raise InterruptedError('planning canceled')
        item=dict(rank=rank,group='object_z',yaw_deg=yaw,home_angle_deg=float(np.degrees(angle)))
        reports.append(item)
        trial_config=deepcopy(c)
        trial_config['temporary_axis_grasp_trial']=False
        trial_config['_axis_trial_hold_descent']=True
        transform_trial=transform.copy();transform_trial[:3,:3]=obj[:3,:3].T@rotation
        trial_config['grasp_object_from_right_ee']=transform_trial.tolist()
        trial=SimpleNamespace(config=trial_config,robot=r)
        stage='Approach'
        try:
            approach=Planner.plan(trial,stage,q,obj,canceled=canceled)
            item['approach_ik']=True
            stage='Descend'
            Planner.plan(trial,stage,approach['path'][-1],obj,canceled=canceled)
            item['full_path_valid']=True
        except ValueError as e:
            item.update(full_path_valid=False,path_error=str(e),failed_stage=stage)
            continue
        report=dict(candidates=reports,elapsed_ms=(time.perf_counter()-begin)*1000,
                    selected_rank=rank,selected_group='object_z',selected_yaw_deg=yaw)
        print('AXIS_GRASP_TRIAL '+json.dumps(report),flush=True)
        approach['ik_calls']=r.ik_calls;approach['axis_grasp_report']=report
        return approach
    report=dict(candidates=reports,elapsed_ms=(time.perf_counter()-begin)*1000,selected_rank=None)
    print('AXIS_GRASP_TRIAL '+json.dumps(report),flush=True)
    raise ValueError('axis grasp trial: neither 0 nor 180 degree approach+descent is feasible; '+json.dumps(reports))
