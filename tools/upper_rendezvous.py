"""Upper-only fixed-height rendezvous mathematics/planning. No ROS or commands."""
import copy
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from right_only_trial import planning_robot
from async_upper import validate, spline
from deux_task_core.continuous_grasp_preview import jacobian
from deux_task_core.position_first_preview import bounded_joint_velocity
from deux_task_core.live_continuous_preview import roi_contains
from deux_task_core.tracking_policy import inside_xy

DEFAULTS = dict(upper_height_m=.05, ramp_s=.4, follow_s=.8, qualify_s=.5, xy_m=.02, z_m=.01,
                relative_speed_m_s=.02, height_band_m=.015, max_observation_age_s=.5,
                max_feedback_gap_s=.1, wait_timeout_s=120., settle_s=.5)


def validate_settings(p):
    keys=list(DEFAULTS)+['upstream_offset_m']
    if any(k not in p or not np.isfinite(p[k]) or p[k]<=0 for k in keys):raise ValueError('invalid upper trial settings')
    if not .2<=p['ramp_s']<=1.6 or not p['qualify_s']+.2<=p['follow_s']<=2.:raise ValueError('invalid merge/qualification duration')
    if p['xy_m']>.02 or p['z_m']>.01 or p['relative_speed_m_s']>.02:raise ValueError('upper tolerances cannot exceed reviewed bounds')
    if p['height_band_m']>.015 or p['max_observation_age_s']>.5 or p['upstream_offset_m']>.1:raise ValueError('invalid geometry/evidence limits')
    return p


def fixed_observation(obj, velocity, grasp, fixed, height_band=.015):
    obj=np.array(obj,float).copy(); v=np.array(velocity,float).copy()
    if obj.shape!=(4,4) or v.shape!=(4,) or not np.isfinite(obj).all() or not np.isfinite(v).all():
        raise ValueError('invalid observation')
    if abs(v[3])>1e-8 or not .005<=np.linalg.norm(v[:2])<=.2:
        raise ValueError('requires moving straight belt')
    point=obj@np.asarray(grasp); fixed=np.asarray(fixed)
    if height_band is not None and abs(point[2,3]-fixed[2,3])>height_band:raise ValueError('bag outside fixed-height band')
    if Rotation.from_matrix(point[:3,:3]@fixed[:3,:3].T).magnitude()>np.deg2rad(15):
        raise ValueError('bag outside fixed-orientation band')
    raw_z=float(point[2,3]); obj[2,3]+=fixed[2,3]-point[2,3]; v[2]=0.
    return obj,v,raw_z


def arrival(fixed, obj, velocity, grasp, stamp, now):
    if not np.isfinite([stamp,now]).all() or now<stamp:raise ValueError('invalid observation clock')
    v=np.asarray(velocity); p=(obj@grasp)[:2,3]+v[:2]*(now-stamp)
    delta=np.asarray(fixed)[:2,3]-p; speed2=float(v[:2]@v[:2])
    if speed2<.005**2:raise ValueError('stationary target')
    eta=float(delta@v[:2]/speed2)
    return now+eta,float(np.linalg.norm(delta-v[:2]*eta))


def target(fixed, height, velocity, at, ramp_s):
    """Ramp starts t=0. Fixed-point crossing at ramp/2; velocity matches at ramp.
    Position/velocity/acceleration are continuous at both ramp boundaries.
    """
    p=np.array(fixed,float).copy();v=np.asarray(velocity,float)
    u=np.clip(at/ramp_s,0.,1.)
    if at<=0:area=rate=0.
    elif at<ramp_s:area=ramp_s*(u**3-.5*u**4);rate=3*u*u-2*u**3
    else:area=at-ramp_s/2;rate=1.
    p[:2,3]+=v[:2]*area;p[2,3]+=height
    return p,np.r_[v[:2]*rate,0.,0.,0.,0.]


def stationary_path(config, current, endpoint):
    from ready_recovery import compute_ready
    c=copy.deepcopy(config);r=planning_robot(c)
    way=compute_ready(c,current,endpoint)['right_waypoints']
    times=[0.];path=[np.asarray(current).copy()];vel=[np.zeros(28)]
    for right in way[1:]:
        a=path[-1];b=a.copy();b[14:21]=right;delta=b-a
        duration=max(1.,float(np.max(1.875*abs(delta)/np.minimum(r.velocity,.25))),
                     float(np.sqrt(np.max(6*abs(delta)/np.minimum(r.acceleration,.3)))))+.2
        ts=np.linspace(0,duration,int(np.ceil(duration/.02))+1)[1:];u=ts/duration
        offset=times[-1]
        for tt,ss,ds in zip(ts,10*u**3-15*u**4+6*u**5,(30*u**2-60*u**3+30*u**4)/duration):
            times.append(offset+tt)
            path.append(a+ss*delta);vel.append(ds*delta)
    out=dict(times=times,path=np.array(path).tolist(),velocities=np.array(vel).tolist())
    out['min_conveyor_margin_m']=validate(c,out,r)
    return out


def prepare_wait(config,current,seed,fixed):
    # Keep the existing worker API; the measured start determines every route stage.
    from right_clear_approach import plan_right_approach
    target_pose=np.array(fixed).copy();target_pose[2,3]+=config['approach_height']
    return plan_right_approach(config,current,target_pose)


def following_path(config, q, fixed, velocity, policy):
    r=planning_robot(config);q=np.asarray(q,float).copy();qd=np.zeros(28)
    v=np.asarray(velocity);height=config['approach_height'];ramp=policy['ramp_s'];end=ramp+policy['follow_s']
    ts=np.linspace(0,end,int(np.ceil(end/.01))+1);path=[q.copy()];vel=[qd.copy()]
    for a,b in zip(ts[:-1],ts[1:]):
        h=b-a;fk,j=jacobian(r,q);desired,feed=target(fixed,height,v,a,ramp)
        error=np.r_[desired[:3,3]-fk[:3,3],Rotation.from_matrix(desired[:3,:3]@fk[:3,:3].T).as_rotvec()]
        weights=np.array([1.,1.,1.,.2,.2,.2]);jw=j*weights[:,None]
        wanted=np.linalg.solve(jw.T@jw+.015**2*np.eye(7),jw.T@(weights*(feed+np.r_[6*error[:3],4*error[3:]])))
        wanted=bounded_joint_velocity(r,q,wanted);n=qd.copy()
        n[14:21]+=np.clip(wanted-qd[14:21],-.95*r.acceleration[14:21]*h,.95*r.acceleration[14:21]*h)
        q+=(qd+n)*h/2;qd=n;path.append(q.copy());vel.append(qd.copy())
    brake=max(.6,float(np.max(abs(qd)/(.7*r.acceleration))))
    bt=np.linspace(0,brake,int(np.ceil(brake/.01))+1)[1:]
    path=np.vstack([path,q+qd*(bt-bt*bt/(2*brake))[:,None]])
    vel=np.vstack([vel,qd*(1-bt/brake)[:,None]])
    out=dict(times=np.r_[ts,end+bt].tolist(),path=path.tolist(),velocities=vel.tolist(),
             merge_s=ramp,brake_start_s=end,velocity=v.tolist(),fixed_pose=np.asarray(fixed).tolist())
    out['min_conveyor_margin_m']=validate(config,out,r)
    # Hard task-space criteria, including constant Z, instead of warning-only IK residuals.
    curve=spline(out);errors=[]
    for t in np.arange(ramp,end+.001,.02):
        fk=r.ee_fk('right',curve(t));wanted,_=target(fixed,height,v,t,ramp)
        xy=float(np.linalg.norm(fk[:2,3]-wanted[:2,3]));z=abs(float(fk[2,3]-wanted[2,3]))
        if xy>policy['xy_m'] or z>policy['z_m']:raise ValueError('upper path Cartesian tolerance')
        errors.append([xy,z])
    # The whole stop tail also stays above the fixed surface and inside the XY task window.
    points=np.array([r.ee_fk('right',curve(t))[:3,3] for t in np.linspace(0,out['times'][-1],100)])
    if np.max(abs(points[:,2]-(fixed[2,3]+height)))>policy['z_m']:raise ValueError('upper height lost during ramp/braking')
    if not inside_xy(config['tracking_grasp_policy'],points):raise ValueError('upper/braking path leaves workspace')
    out['max_follow_error_m']=np.max(errors,axis=0).tolist()
    return out


def dispatch_checks(plan, fixed, grasp, obj, v, stamp, now, geometry, policy):
    if not 0<=now-stamp<=policy['max_observation_age_s']:raise ValueError('stale observation')
    at,lateral=arrival(fixed,obj,v,grasp,stamp,now)
    if lateral>policy['xy_m']/2:raise ValueError('lane misses waiting point')
    if np.linalg.norm(np.asarray(plan['velocity'])[:2]-v[:2])>.005:raise ValueError('velocity changed during planning')
    start=at-policy['ramp_s']/2
    if start-now<.25:raise ValueError('too late for velocity merge')
    if start-now>3.:raise ValueError('target too far upstream')
    points=[(obj@grasp)[:3,3]+v[:3]*(start-stamp+t) for t in np.linspace(0,plan['times'][-1],30)]
    if not roi_contains(geometry,points):raise ValueError('forecast leaves observation ROI')
    return start,at,lateral


class Qualification:
    """Fresh feedback only, causal velocity fit; no success after any evidence loss."""
    def __init__(self,policy):
        self.p=policy;self.samples=deque();self.since=None;self.passed=False;self.invalid=False;self.last_stamp=None
    def update(self,stamp,hand,bag,v,eligible=True):
        if self.last_stamp is not None and stamp<=self.last_stamp:
            self.since=None
            return None
        if self.last_stamp is not None and stamp-self.last_stamp>self.p['max_feedback_gap_s']:self.since=None;self.samples.clear()
        self.last_stamp=stamp;self.samples.append((stamp,np.asarray(hand).copy()))
        while self.samples and stamp-self.samples[0][0]>.16:self.samples.popleft()
        if len(self.samples)<4 or stamp-self.samples[0][0]<.06:return None
        tt=np.array([s-stamp for s,_ in self.samples]);pp=np.array([p for _,p in self.samples])
        hv=np.linalg.lstsq(np.column_stack([tt,np.ones(len(tt))]),pp,rcond=None)[0][0]
        delta=np.asarray(hand)-bag;xy=float(np.linalg.norm(delta[:2]));z=abs(float(delta[2]));speed=float(np.linalg.norm(hv[:2]-v[:2]))
        good=eligible and xy<=self.p['xy_m'] and z<=self.p['z_m'] and speed<=self.p['relative_speed_m_s']
        self.since=(stamp if self.since is None else self.since) if good else None
        if self.since is not None and stamp-self.since>=self.p['qualify_s']:self.passed=True
        return dict(feedback_stamp=stamp,hand=hand.tolist(),bag_upper=bag.tolist(),hand_velocity=hv.tolist(),bag_velocity=v.tolist(),xy_m=xy,z_m=z,relative_xy_speed_m_s=speed,qualified=bool(good),qualified_duration_s=0. if self.since is None else stamp-self.since)


class Settling:
    """Causal position-based joint speed avoids quantized motor velocity chatter."""
    def __init__(self,point,duration=.5):
        self.point=np.asarray(point);self.duration=duration;self.rows=deque();self.since=None
    def update(self,stamp,q,hand):
        if self.rows and stamp<self.rows[-1][0]:raise ValueError('feedback clock moved backwards')
        if self.rows and stamp==self.rows[-1][0]:return False,None
        if self.rows and stamp-self.rows[-1][0]>.1:self.rows.clear();self.since=None
        self.rows.append((stamp,np.asarray(q).copy(),np.asarray(hand).copy()))
        while self.rows and stamp-self.rows[0][0]>.2:self.rows.popleft()
        if len(self.rows)<5 or stamp-self.rows[0][0]<.12:return False,None
        t=np.array([x[0]-stamp for x in self.rows]);angles=np.array([x[1] for x in self.rows]);points=np.array([x[2] for x in self.rows])
        speed=np.linalg.lstsq(np.column_stack([t,np.ones(len(t))]),angles,rcond=None)[0][0]
        error=float(np.max(np.linalg.norm(points-self.point,axis=1)))
        good=error<=.01 and np.max(abs(speed))<.03 and np.max(np.ptp(angles,axis=0))<.01
        self.since=(stamp if self.since is None else self.since) if good else None
        e=dict(feedback_stamp=stamp,position_error_m=error,joint_speed_from_positions=speed.tolist(),qualified=bool(good),duration_s=0. if self.since is None else stamp-self.since)
        return bool(self.since is not None and stamp-self.since>=self.duration),e
