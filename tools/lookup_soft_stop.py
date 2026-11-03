"""Choose certified joint-wise braking to reduce end-effector travel."""
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from lookup_fast_scan import certify_replay


def build_path_stop(plan, elapsed, robot, ready, canceled=lambda:False):
    source=CubicHermiteSpline(plan['times_s'],plan['positions_rad'],plan['velocities_rad_s'])
    if source.c.shape[-1]!=14:raise ValueError('soft stop requires both arm trajectories')
    remaining=float(source.x[-1])-elapsed
    last='insufficient remaining replay for soft stop'
    # Prefer stronger 0.6 s braking; lengthen only if certification requires it.
    for duration in (.6,.8,1.,1.2,1.4):
        if elapsed<4. or duration/2>=remaining-.01:continue
        t=np.linspace(0.,duration,round(duration/.01)+1);u=t/duration
        # Path speed changes smoothly from 1 to 0, with zero slope at both ends.
        path_time=elapsed+duration*(u-2.5*u**4+3*u**5-u**6)
        rate=1-10*u**3+15*u**4-6*u**5
        q=source(path_time);v=source(path_time,1)*rate[:,None];v[-1]=0.
        data=dict(times_s=t.tolist(),positions_rad=q.tolist(),velocities_rad_s=v.tolist())
        try:
            # Validator's conveyor suffix starts at t=4; shift only its time origin.
            checked=dict(data,times_s=(t+4.).tolist())
            margin=certify_replay(robot,ready,checked,canceled)
        except ValueError as error:last=str(error);continue
        data.update(duration_s=duration,source_elapsed_s=elapsed,source_stop_s=float(path_time[-1]),
                    minimum_clearance_m=margin)
        return data
    raise ValueError('no certified soft-stop trajectory: '+last)


def joint_stop(source,elapsed,durations):
    durations=np.asarray(durations,float);end=float(max(durations))
    times=np.unique(np.round(np.r_[np.linspace(0.,end,int(np.ceil(end/.01))+1),durations],10))
    u=np.minimum(times[:,None]/durations,1.)
    path_times=elapsed+durations*(u-2.5*u**4+3*u**5-u**6)
    rates=1-10*u**3+15*u**4-6*u**5
    columns=np.arange(14)
    q=source(path_times)[:,columns,columns]
    v=source(path_times,1)[:,columns,columns]*rates
    v[times[:,None]>=durations]=0.
    return dict(times_s=times.tolist(),positions_rad=q.tolist(),velocities_rad_s=v.tolist(),
                duration_s=end,joint_stop_durations_s=durations.tolist(),source_elapsed_s=elapsed,
                source_stop_times_s=(elapsed+durations/2).tolist())


def dynamic_ratios(robot,data):
    """Exact velocity/acceleration extrema of the transmitted cubic segments."""
    ts=np.asarray(data['times_s']);s=CubicHermiteSpline(ts,data['positions_rad'],data['velocities_rad_s'])
    a,b,c,_=s.c;dt=np.diff(ts)[:,None]
    u=np.zeros_like(a);np.divide(-b,3*a,out=u,where=abs(a)>1e-14);u=np.clip(u,0,dt)
    velocity=np.max(abs(np.stack([c,(3*a*dt+2*b)*dt+c,(3*a*u+2*b)*u+c])),axis=(0,1))
    acceleration=np.max(np.maximum(abs(2*b),abs(6*a*dt+2*b)),axis=0)
    idx=np.r_[0:7,14:21]
    return np.maximum(velocity/robot.velocity[idx],acceleration/robot.acceleration[idx])


def travel(robot,ready,data):
    """Sample transmitted spline at <=10ms; batch exact URDF FK for both EEs."""
    from scipy.spatial.transform import Rotation
    from deux_task_core.model import JOINT_NAMES
    end=data['times_s'][-1];t=np.linspace(0.,end,int(np.ceil(end/.01))+1)
    q=np.tile(ready,(len(t),1));q[:,np.r_[0:7,14:21]]=CubicHermiteSpline(data['times_s'],data['positions_rad'],data['velocities_rad_s'])(t)
    values=dict(zip(JOINT_NAMES,q.T));metrics={}
    for side in ('left','right'):
        transforms=np.tile(np.eye(4),(len(t),1,1))
        for name,joint in robot.ee_chains[side]:
            motion=np.tile(np.eye(4),(len(t),1,1))
            if joint['kind']!='fixed':
                value=np.broadcast_to(values.get(name,robot.fixed.get(name)),(len(t),))
                if joint['kind'] in ('revolute','continuous'):motion[:,:3,:3]=Rotation.from_rotvec(value[:,None]*joint['axis']).as_matrix()
                else:motion[:,:3,3]=value[:,None]*joint['axis']
            transforms=transforms@joint['origin']@motion
        xyz=transforms[:,:3,3]
        metrics[side+'_travel_m']=float(np.linalg.norm(np.diff(xyz,axis=0),axis=1).sum())
        metrics[side+'_max_displacement_m']=float(np.linalg.norm(xyz-xyz[0],axis=1).max())
    return metrics


def build_soft_stop(plan,elapsed,robot,ready,canceled=lambda:False):
    # Keep the already-certified common-path stop as a fallback and comparison.
    baseline=build_path_stop(plan,elapsed,robot,ready,canceled)
    baseline_metrics=travel(robot,ready,baseline)
    baseline.update(profile='common_path_fallback',ee_travel=baseline_metrics)
    source=CubicHermiteSpline(plan['times_s'],plan['positions_rad'],plan['velocities_rad_s'])
    max_duration=min(1.4,2*(source.x[-1]-elapsed-.011))
    grid=[d for d in (.2,.3,.4,.5,.6,.8,1.,1.2,1.4) if d<=max_duration]
    shortest=np.full(14,np.nan)
    for duration in grid:
        if canceled():raise InterruptedError('joint-wise stop planning canceled')
        data=joint_stop(source,elapsed,np.full(14,duration))
        valid=dynamic_ratios(robot,data)<=1.000001
        shortest[np.isnan(shortest)&valid]=duration
    if np.isnan(shortest).any():return baseline
    candidates=[]
    # Wrist and proximal joints may stop at different times. Search a bounded
    # family: slower proximal braking can counter wrist-induced EE movement.
    for proximal_scale in (1.,1.25,1.5,2.):
        for wrist_scale in (1.,1.25,1.5):
            if canceled():raise InterruptedError('joint-wise stop planning canceled')
            durations=np.minimum(shortest*proximal_scale,max_duration)
            durations[[5,6,12,13]]=np.minimum(shortest[[5,6,12,13]]*wrist_scale,max_duration)
            data=joint_stop(source,elapsed,durations)
            if np.max(dynamic_ratios(robot,data))>1.000001:continue
            metrics=travel(robot,ready,data)
            # Neither EE may travel farther or swing farther than the old stop.
            if any(metrics[k]>baseline_metrics[k]+1e-6 for k in metrics):continue
            if metrics['right_travel_m']>=baseline_metrics['right_travel_m']-1e-5:continue
            candidates.append((metrics['right_travel_m']+.25*metrics['left_travel_m'],data,metrics))
    for _,data,metrics in sorted(candidates,key=lambda row:row[0]):
        if canceled():raise InterruptedError('joint-wise stop planning canceled')
        try:
            checked=dict(data,times_s=(np.asarray(data['times_s'])+4.).tolist())
            margin=certify_replay(robot,ready,checked,canceled)
        except ValueError:continue
        data.update(profile='jointwise_min_ee_travel',ee_travel=metrics,baseline_ee_travel=baseline_metrics,
                    baseline_duration_s=baseline['duration_s'],minimum_clearance_m=margin,
                    objective='minimum right EE travel + 0.25 left EE travel among certified candidates; neither EE worsens',
                    joint_dynamic_ratios=dynamic_ratios(robot,data).tolist())
        return data
    return baseline
