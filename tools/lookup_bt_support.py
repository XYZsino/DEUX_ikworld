"""Pure BT handoff and fresh preparation helpers. No ROS imports or commands."""
from copy import deepcopy
import numpy as np
from build_right_grasp_grid import ROOT
from deux_task_core.model import RobotModel
from deux_task_core.prepare_route import knots
from lookup_conveyor import validate_conveyor_path


def prepare_plan(config,ready,actual_arms,canceled=lambda:False,velocity_cap=.15,acceleration_cap=.3):
    if not np.isfinite([velocity_cap,acceleration_cap]).all() or min(velocity_cap,acceleration_cap)<=0:
        raise ValueError('positive finite ready velocity/acceleration caps required')
    start=np.asarray(ready,float).copy();arms=np.asarray(actual_arms,float)
    if arms.shape!=(14,) or not np.isfinite(arms).all():raise ValueError('finite measured arms required')
    start[:7]=arms[:7];start[14:21]=arms[7:]
    local=deepcopy(config);local['home_joints']=np.asarray(ready).tolist()
    local['robot']['collision_check_enabled']=False
    local['obstacles']=[]
    robot=RobotModel(local['robot'])
    if np.max(abs(np.r_[start[:7]-ready[:7],start[14:21]-ready[14:21]]))<=.01:
        return dict(times=np.array([0.,.25]),path=np.array([start,start]),minimum_clearance=float(validate_conveyor_path(robot,[start],canceled)),no_motion=True)
    # Restore the installation LEFT home as well as right ready after scanner use.
    # Existing prepare route moves right first, then restores left at right-ready; conveyor checked for BOTH arms.
    goal=start.copy();goal[:7]=ready[:7];goal[14:21]=ready[14:21]
    q=np.asarray(knots(local,start,goal,robot,canceled))
    # Same stopped cubic geometry; retime within both requested and robot limits.
    keep=[0]+[i for i in range(1,len(q)) if np.max(abs(q[i]-q[i-1]))>1e-10]
    if len(keep)<2:keep=[0,len(q)-1]
    points=q[keep];times=[0.]
    indices=np.r_[np.arange(7),np.arange(14,21)]
    velocity=np.minimum(robot.velocity[indices],velocity_cap)
    acceleration=np.minimum(robot.acceleration[indices],acceleration_cap)
    for a,b in zip(points[:-1],points[1:]):
        delta=abs(b[indices]-a[indices])
        times.append(times[-1]+max(.15,float(np.max(1.5*delta/velocity)),float(np.sqrt(np.max(6*delta/acceleration)))))
    margin=float(validate_conveyor_path(robot,points,canceled))
    return dict(times=np.array(times),path=points,minimum_clearance=margin,no_motion=False)


def validate_handoff(result,request_id,requested_target=''):
    if not result.success or not result.path_completed:raise ValueError('lookup replay did not complete')
    if result.request_id!=request_id:raise ValueError('lookup result request mismatch')
    if not result.target_id or not result.cell_id:raise ValueError('lookup result identity missing')
    if requested_target and result.target_id!=requested_target:raise ValueError('lookup changed requested target')
    if result.grasp_mode not in ('normal','transverse_axis_locked'):raise ValueError('unknown lookup grasp mode')
    if result.contact_grasp.header.frame_id!='base_link':raise ValueError('lookup result frame mismatch')
    if result.physical_grasp_verified:raise ValueError('stored motion cannot certify physical grasp')
    return result.target_id
