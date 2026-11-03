"""Pure BT handoff and fresh preparation helpers. No ROS imports or commands."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import numpy as np
from build_right_grasp_grid import ROOT
from deux_task_core.model import RobotModel
from deux_task_core.prepare_route import knots
from lookup_conveyor import validate_conveyor_path


def attention_ready_knots(config,start,goal):
    """Mandatory compact route near attention; None leaves other returns unchanged.

    No shortcut or random-search fallback: failure must not restore the wide swing.
    Full conveyor and joint certification is performed by prepare_plan below.
    """
    data=json.loads((ROOT/'calibration/attention_ready_route.json').read_text())
    way=np.asarray(data['right_waypoints'],float)
    if way.shape!=(4,7) or not np.isfinite(way).all():
        raise ValueError('invalid compact attention route')
    if np.max(abs(start[14:21]-way[0]))>float(data['start_tolerance_rad']):
        return None
    if hashlib.sha256(Path(config['robot']['urdf_path']).read_bytes()).hexdigest()!=data['urdf_sha256']:
        raise ValueError('compact attention route model changed; regenerate before use')
    if np.max(abs(way[-1]-goal[14:21]))>1e-6:
        raise ValueError('compact attention route ready target changed; regenerate before use')
    # Fixed progress preserves the left arm's straight joint-space path while
    # both arms start and finish together. Independent of velocity caps.
    progress=np.r_[0.,np.cumsum(np.max(abs(np.diff(way,axis=0)),axis=1))]
    progress/=progress[-1]
    result=[start.copy()]
    for arm,fraction in zip(way[1:],progress[1:]):
        q=start.copy();q[14:21]=arm
        q[:7]=start[:7]+fraction*(goal[:7]-start[:7]);result.append(q)
    return result


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
    # Attention starts move both arms together; other returns keep the existing route.
    goal=start.copy();goal[:7]=ready[:7];goal[14:21]=ready[14:21]
    compact=attention_ready_knots(local,start,goal)
    q=np.asarray(compact if compact is not None else knots(local,start,goal,robot,canceled))
    return time_prepare_path(robot,q,canceled,velocity_cap,acceleration_cap)


def time_prepare_path(robot,q,canceled,velocity_cap,acceleration_cap):
    """Certify both arms and time stopped cubic segments with joint limits."""
    if not np.isfinite([velocity_cap,acceleration_cap]).all() or min(velocity_cap,acceleration_cap)<=0:
        raise ValueError('positive finite ready velocity/acceleration caps required')
    q=np.asarray(q,float)
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


def attention_plan(config,ready,actual_arms,canceled=lambda:False,velocity_cap=.15,acceleration_cap=.3):
    """Current -> ready -> exact reverse of canonical simultaneous preparation.

    Only arm joints are commanded; the driver retains existing finger commands.
    Failure of any segment rejects the entire plan before dispatch.
    """
    if not np.isfinite([velocity_cap,acceleration_cap]).all() or min(velocity_cap,acceleration_cap)<=0:
        raise ValueError('positive finite ready velocity/acceleration caps required')
    ready=np.asarray(ready,float);arms=np.asarray(actual_arms,float)
    if arms.shape!=(14,) or not np.isfinite(arms).all():raise ValueError('finite measured arms required')
    indices=np.r_[np.arange(7),np.arange(14,21)]
    attention=ready.copy();attention[indices]=0.
    local=deepcopy(config);local['robot']['collision_check_enabled']=False;local['obstacles']=[]
    robot=RobotModel(local['robot'])
    # Verify the stored model and target even when no motion is needed.
    forward=np.asarray(attention_ready_knots(local,attention,ready))
    if np.max(abs(arms))<=.01:
        actual=attention.copy();actual[indices]=arms
        return dict(times=np.array([0.,.25]),path=np.array([actual,actual]),
                    minimum_clearance=float(validate_conveyor_path(robot,[actual],canceled)),no_motion=True)
    prefix=prepare_plan(config,ready,arms,canceled,velocity_cap,acceleration_cap)
    # Include the exact ready knot to bridge residual measured endpoint error.
    path=np.vstack([prefix['path'],forward[::-1]])
    return time_prepare_path(robot,path,canceled,velocity_cap,acceleration_cap)


def validate_handoff(result,request_id,requested_target=''):
    if not result.success or not result.path_completed:raise ValueError('lookup replay did not complete')
    if result.request_id!=request_id:raise ValueError('lookup result request mismatch')
    if not result.target_id or not result.cell_id:raise ValueError('lookup result identity missing')
    if requested_target and result.target_id!=requested_target:raise ValueError('lookup changed requested target')
    if result.grasp_mode not in ('normal','transverse_axis_locked'):raise ValueError('unknown lookup grasp mode')
    if result.contact_grasp.header.frame_id!='base_link':raise ValueError('lookup result frame mismatch')
    if result.physical_grasp_verified:raise ValueError('stored motion cannot certify physical grasp')
    return result.target_id
