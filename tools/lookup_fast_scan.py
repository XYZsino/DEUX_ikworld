"""Preserve the right grasp-to-via curve; shorten only via-to-scan transport."""
from copy import deepcopy
import hashlib,json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline,make_interp_spline
from lookup_conveyor import arm_clearance
ROOT=Path(__file__).resolve().parents[1]
PROFILE='via_preserved_fast_scan'
REPORT=ROOT/'artifacts/scan-speed-search/library-validation.json'


class ScanTailCurve:
    def __init__(self,original,tail,via,end,left):
        self.original=original;self.tail=tail;self.via=via;self.left_curve=left
        self.x=np.array([0.,4.,via,end])
    def __call__(self,t,nu=0):
        values=np.asarray(t);flat=values.reshape(-1);out=np.empty((len(flat),7));mask=flat<=self.via
        out[mask]=self.original(flat[mask],nu);out[~mask]=self.tail(flat[~mask]-self.via,nu)
        return out.reshape(values.shape+(7,))


def compile_tail(library,key,config,left_target,tail_duration=None,reuse_clearance=None):
    from lookup_lift_scan import compile_path,LeftScanCurve
    from deux_task_core.model import ARM_NAMES
    if left_target is None:raise ValueError('parallel scan requires a left target')
    legacy,original,old=compile_path(library,key,config,left_target=left_target,reuse_clearance=0.)
    via=old['waypoint_at_s'];old_tail=old['duration_s']-via
    candidates=[float(tail_duration)] if tail_duration is not None else np.unique(np.r_[np.arange(1.8,old_tail,.1),old_tail])
    last_error='no timing candidate'
    for duration in candidates:
        if not 0<duration<=old_tail+1e-8:raise ValueError('tail duration must not exceed original')
        tail=make_interp_spline([0.,duration],[original(via),original(old['duration_s'])],k=5,
            bc_type=([(1,original(via,1)),(2,original(via,2))],[(1,np.zeros(7)),(2,np.zeros(7))]))
        left=LeftScanCurve(library.ready[:7],np.asarray(left_target),via+duration,begin=via)
        curve=ScanTailCurve(original,tail,via,via+duration,left)
        times=np.unique(np.round(np.r_[np.asarray(legacy['times_s'])[np.asarray(legacy['times_s'])<=via],
            via,np.arange(via,via+duration,.02),via+duration],9))
        positions=np.c_[left(times),curve(times)];velocities=np.c_[left(times,1),curve(times,1)]
        plan=deepcopy(legacy);plan.update(times_s=times.tolist(),positions_rad=positions.tolist(),
            velocities_rad_s=velocities.tolist(),joint_names=list(ARM_NAMES))
        plan['events']=dict(plan['events'],lift_end_s=via,stopped_s=via+duration)
        try:margin=certify_replay(library.robot,library.ready,plan) if reuse_clearance is None else float(reuse_clearance)
        except ValueError as error:last_error=str(error);continue
        info=dict(scan_completed=True,timing_profile=PROFILE,duration_s=via+duration,waypoint_at_s=via,
            original_duration_s=old['duration_s'],tail_duration_s=duration,minimum_conveyor_clearance_m=margin,
            contact_s=3.,unchanged_prefix_until_s=via,left_scan_parallel=True,left_scan_start_s=via,
            left_scan_completed_s=via+duration,left_scan_target_rad=np.asarray(left_target).tolist())
        return plan,curve,info
    raise ValueError('no bounded fast scan tail: '+last_error)


def fingerprint(library,config,left_target):
    files=['calibration/conveyor_visible_surface.json','tools/lookup_lift_scan.py','tools/lookup_fast_scan.py',
           'tools/lookup_conveyor.py','src/deux_task_core/deux_task_core/model.py',
           'src/deux_task_core/deux_task_core/conveyor_screen.py']
    data=dict(profile=PROFILE,files={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in files},
        urdf=hashlib.sha256(Path(config['robot']['urdf_path']).read_bytes()).hexdigest(),
        manifest=library.manifest,ready=library.ready.tolist(),left_target=np.asarray(left_target).tolist(),
        right_target=config['scan_right_joints'],velocity=library.robot.velocity.tolist(),
        acceleration=library.robot.acceleration.tolist(),fixed_joints=library.robot.fixed,
        joint_limit_tolerance=library.robot.joint_limit_tolerance)
    return hashlib.sha256(json.dumps(data,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def certify_replay(robot,ready,plan,canceled=lambda:False):
    """Exact cubic joint extrema plus continuous conveyor bound for sent spline.

    Conveyor distance is 1-Lipschitz. Chain radius * sum joint speed bounds
    the displacement between samples, covering the whole interpolated curve.
    """
    indices=np.r_[0:7,14:21]
    times=np.asarray(plan['times_s']);q=np.asarray(plan['positions_rad']);v=np.asarray(plan['velocities_rad_s'])
    if q.shape[1]!=14:raise ValueError('fast scan requires both arm targets')
    spline=CubicHermiteSpline(times,q,v);a,b,c,d=spline.c
    dt=np.diff(times)[:,None]
    def pos(u):return ((a*u+b)*u+c)*u+d
    def vel(u):return (3*a*u+2*b)*u+c
    def acc(u):return 6*a*u+2*b
    # Roots of q'(u) for position extrema, and q''(u) for speed extrema.
    disc=4*b*b-12*a*c
    roots=[]
    for sign in (-1,1):
        root=np.zeros_like(a)
        quadratic=(abs(a)>1e-14)&(disc>=0)
        np.divide(-2*b+sign*np.sqrt(np.maximum(disc,0)),6*a,out=root,where=quadratic)
        linear=(abs(a)<=1e-14)&(abs(b)>1e-14)
        np.divide(-c,2*b,out=root,where=linear)
        roots.append(np.clip(root,0,dt))
    positions=np.stack([pos(0),pos(dt),*[pos(u) for u in roots]])
    if np.any(positions<robot.limits[indices,0]-robot.joint_limit_tolerance) or np.any(positions>robot.limits[indices,1]+robot.joint_limit_tolerance):
        raise ValueError('transmitted cubic position limit')
    u=np.zeros_like(a);np.divide(-b,3*a,out=u,where=abs(a)>1e-14);u=np.clip(u,0,dt)
    vmax=np.max(abs(np.stack([vel(0),vel(dt),vel(u)])),axis=0)
    amax=np.maximum(abs(acc(0)),abs(acc(dt)))
    if np.max(vmax/robot.velocity[indices])>1.000001 or np.max(amax/robot.acceleration[indices])>1.000001:
        raise ValueError('transmitted cubic velocity/acceleration limit')
    suffix=times[:-1]>=4.-1e-9
    margin=float('inf')
    for side,part in [('left',slice(0,7)),('right',slice(7,14))]:
        radius=sum(np.linalg.norm(j['origin'][:3,3]) for _,j in robot.ee_chains[side])+.01
        speed_bound=radius*float(np.max(vmax[suffix,part].sum(axis=1)))
        for step in (.02,.01,.005,.0025):
            ts=np.linspace(4.,times[-1],int(np.ceil((times[-1]-4.)/step))+1)
            bound=.5*speed_bound*float(np.max(np.diff(ts)))
            minimum=float('inf')
            for arms in spline(ts):
                if canceled():raise InterruptedError('fast scan audit canceled')
                full=np.asarray(ready).copy();full[indices]=arms
                minimum=min(minimum,arm_clearance(robot,full,side))
                if minimum<0:raise ValueError(side+' transmitted cubic conveyor collision')
            if minimum>bound:
                margin=min(margin,minimum-bound);break
        else:raise ValueError(side+' transmitted cubic clearance not certified')
    return margin


def load_audited(library,config,left_target):
    report=json.loads(REPORT.read_text())
    if fingerprint(library,config,left_target)!=report['fingerprint']:
        raise ValueError('fast scan audit inputs changed; regenerate scan-speed-search/library-validation.json')
    accepted={};rejected={}
    for key in library.entries:
        row=report['accepted'].get(key)
        if row is None:
            if key not in report['rejected']:raise ValueError('fast scan audit missing cell '+key)
            rejected[key]=report['rejected'][key];continue
        accepted[key]=compile_tail(library,key,config,left_target=left_target,
            tail_duration=row['tail_duration_s'],reuse_clearance=row['minimum_conveyor_clearance_m'])
    if not accepted:raise ValueError('no audited fast scan cells')
    return accepted,rejected
