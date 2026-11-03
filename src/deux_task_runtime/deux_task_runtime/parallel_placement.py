"""Single time base for reader withdrawal and existing fixed-bin right motion."""
from copy import deepcopy
import numpy as np
from deux_task_core.planning import Planner,hermite


def validate_parallel(config,t,path,phases):
    p=config.get('placement',{})
    if not (p.get('enabled') and p.get('fixed_joint_motion') and p.get('continuous_motion')):
        raise ValueError('parallel placement requires existing continuous fixed-bin policy')
    t=np.asarray(t,float);q=np.asarray(path,float);ph=np.asarray(phases,float)
    if q.shape!=(len(t),28) or len(t)<2 or not np.isfinite(q).all() or not np.isfinite(t).all() or t[0]!=0 or np.any(np.diff(t)<=0):raise ValueError('invalid parallel path')
    if ph.shape!=(3,) or not np.isfinite(ph).all() or ph[0]<=0 or np.any(np.diff(ph)<=0) or abs(ph[-1]-t[-1])>1e-6 or abs(ph[1]-ph[0]-config['hand_open_duration_s'])>1e-6:raise ValueError('invalid parallel release schedule')
    clear=np.asarray(p['clear_left_joints']);left=q[:,:7]
    if np.any(left<np.minimum(left[0],clear)-1e-8) or np.any(left>np.maximum(left[0],clear)+1e-8):raise ValueError('reader withdrawal overshoot')
    if np.any(np.diff(left,axis=0)*np.sign(clear-left[0])<-1e-8):raise ValueError('reader withdrawal reverses direction')
    if not np.allclose(left[t>=ph[0]-1e-6],clear,atol=1e-8):raise ValueError('reader must be clear before opening')
    arm=np.r_[np.arange(7),np.arange(14,21)];hold=q[(t>=ph[0]-1e-6)&(t<=ph[1]+1e-6)][:,arm]
    if len(hold)<2 or not np.allclose(hold,hold[0],atol=1e-8):raise ValueError('opening requires stationary arms')
    release=np.array([np.interp(ph[0],t,q[:,i]) for i in range(28)])
    if not np.allclose(release[:7],clear,atol=1e-8):raise ValueError('reader must be clear at opening boundary')
    if not np.allclose(release[14:21],p['fixed_joint_waypoints']['Place'][-1],atol=1e-7):raise ValueError('parallel release target mismatch')
    if not np.allclose(q[-1,14:21],p['fixed_joint_waypoints']['Retreat'][-1],atol=1e-7):raise ValueError('parallel retreat target mismatch')


def build_parallel(config,actual,canceled=lambda:False):
    p=config.get('placement',{})
    if not (p.get('enabled') and p.get('fixed_joint_motion') and p.get('continuous_motion')):
        raise ValueError('parallel placement requires existing continuous fixed-bin policy')
    c=deepcopy(config);c['ik_backend']='scipy' # Existing fixed joint paths need no IK.
    planner=Planner(c);robot=planner.robot;start=np.asarray(actual,float).copy()
    if start.shape!=(28,) or not np.isfinite(start).all():raise ValueError('fresh measured joints required')
    if c.get('hand_command_only',False):
        start[7:14]=c['hand_closed'][:7];start[21:28]=c['hand_closed'][7:]
    if np.any(start<robot.limits[:,0]-robot.joint_limit_tolerance) or np.any(start>robot.limits[:,1]+robot.joint_limit_tolerance):raise ValueError('measured joint outside limits')
    clear=np.asarray(c['placement']['clear_left_joints'])
    virtual=start.copy();virtual[:7]=clear
    right=planner.plan('PlaceApproach',virtual,np.eye(4),canceled=canceled)
    rt=np.asarray(right['times']);rq=np.asarray(right['path']);ph=np.asarray(right['phase_end_times'])
    left_start=rq[0].copy();left_start[:7]=start[:7];left_goal=left_start.copy();left_goal[:7]=clear
    scale=c['placement'].get('joint_motion_scale',1.)
    lt,_=hermite([left_start,left_goal],robot.velocity*scale,robot.acceleration*scale**2,c['sample_period'])
    sequential_left=float(lt[-1])
    minimum_left=max(sequential_left,float(np.max(1.875*abs(clear-start[:7])/(robot.velocity[:7]*scale))))
    stretch=max(1.,minimum_left/ph[0]);dt=c['sample_period']
    for _ in range(12):
        release=float(ph[0]*stretch);left_duration=max(minimum_left,.65*release)
        new_ph=ph+(release-ph[0]);end=float(new_ph[-1])
        # Keep the original sample grid when possible. Never insert nearly
        # coincident ticks: they amplify finite-difference acceleration noise.
        if abs(stretch-1.)<1e-12:
            t=rt.copy();q=rq.copy()
        else:
            approach=np.linspace(0,release,int(np.ceil(release/dt))+1)
            t=np.r_[approach,release+(rt[rt>ph[0]+1e-8]-ph[0])]
            old=np.where(t<=release,t/stretch,ph[0]+t-release)
            q=np.stack([np.interp(old,rt,rq[:,i]) for i in range(28)],axis=1)
        u=np.clip(t/left_duration,0,1);smooth=10*u**3-15*u**4+6*u**5
        q[:,:7]=start[:7]+smooth[:,None]*(clear-start[:7])
        if canceled():raise InterruptedError('parallel placement canceled')
        validate_parallel(c,t,q,new_ph)
        v=np.diff(q,axis=0)/np.diff(t)[:,None]
        a=np.diff(np.vstack([np.zeros(28),v,np.zeros(28)]),axis=0)/np.r_[np.diff(t),np.diff(t)[-1]][:,None]
        vr=float(np.max(abs(v)/robot.velocity));ar=float(np.max(abs(a)/robot.acceleration))
        if vr<=1.001 and ar<=1.01:break
        stretch*=max(1.05,vr*1.02,np.sqrt(ar)*1.02)
    else:raise ValueError('parallel joint dynamics exceed limits')
    return dict(times=t,path=q,phase_end_times=new_ph,minimum_clearance=right['minimum_clearance'],ik_calls=0,
        report=dict(left_retreat_s=left_duration,right_approach_release_s=release,total_s=end,
                    sequential_left_duration_s=sequential_left,left_profile='zero-velocity/acceleration quintic',original_right_total_s=float(rt[-1]),
                    right_approach_stretch=stretch,simultaneous_start=True,velocity_limit_ratio=vr,acceleration_limit_ratio=ar,stationary_open_s=float(ph[1]-ph[0])))
