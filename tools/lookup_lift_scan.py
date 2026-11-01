"""Compile a checked ready/contact/lift/right-via/scan replay; no ROS commands."""
from copy import deepcopy,copy
import numpy as np
from scipy.interpolate import make_interp_spline
from deux_task_core.model import ARM_NAMES
from lookup_conveyor import validate_conveyor_path

# Accepted FK pose: (0.660011, -0.259993, 1.234997) in base_link.
VIA=np.array([.9541862466872183,.7521680756611958,-.24054751952090306,-1.213402783034755,-.9764438201463088,-.4854183639036923,.813772933694618])

class JoinedCurve:
    def __init__(self,prefix,suffix,end):self.prefix=prefix;self.suffix=suffix;self.x=np.array([0.,4.,end])
    def __call__(self,t,nu=0):
        values=np.asarray(t);flat=values.reshape(-1);out=np.empty((len(flat),7));mask=flat<=4.
        out[mask]=self.prefix(flat[mask],nu);out[~mask]=self.suffix(flat[~mask]-4.,nu)
        return out.reshape(values.shape+(7,))


class LeftScanCurve:
    def __init__(self,start,goal,end):
        self.begin=4.;self.end=float(end)
        z=np.zeros(7)
        self.curve=make_interp_spline([self.begin,self.end],[start,goal],k=5,
            bc_type=([(1,z),(2,z)],[(1,z),(2,z)]))
    def __call__(self,t,nu=0):
        values=np.asarray(t);result=self.curve(np.clip(values,self.begin,self.end),nu)
        if nu:result=np.where(((values<self.begin)|(values>self.end))[...,None],0.,result)
        return result


def compile_path(library,key,config,canceled=lambda:False,left_target=None,reuse_clearance=None):
    robot=library.robot;original=library.curves[key];start=original(4.)
    v=original(4.,1);a=original(np.nextafter(4.,-np.inf),2)
    # Keep the contact deadline and first second of lift exactly as stored.
    # Only post-lift transport can be stretched if another cell requires it.
    for duration in np.arange(4.,8.01,.25):
        suffix=make_interp_spline([0.,duration/2,duration],np.array([start,VIA,config['scan_right_joints']]),k=5,
            bc_type=([(1,v),(2,a)],[(1,np.zeros(7)),(2,np.zeros(7))]))
        dense=np.linspace(0.,duration,int(np.ceil(duration/.01))+1);q=suffix(dense)
        if np.any(q<robot.limits[14:21,0]) or np.any(q>robot.limits[14:21,1]):continue
        if np.max(abs(suffix(dense,1))/robot.velocity[14:21])>.8:continue
        if np.max(abs(suffix(dense,2))/robot.acceleration[14:21])>.64:continue
        break
    else:raise ValueError('no bounded lift-to-scan connection within transport duration budget')
    curve=JoinedCurve(original,suffix,4.+duration)
    t=np.r_[np.asarray(library.entries[key]['times_s']),np.arange(5.02,4.+duration,.02),4.+duration]
    # Replace old suffix; ensure the boundary at 4s is explicit.
    t=np.unique(np.round(np.r_[t[t<=4.],np.arange(0.,4.+duration,.02),4.+duration],9));q=curve(t)
    full=np.tile(library.ready,(len(t),1));full[:,14:21]=q
    curve.left_curve=None
    if left_target is not None:
        curve.left_curve=LeftScanCurve(library.ready[:7],np.asarray(left_target,float),4.+duration/2)
        left_dense=curve.left_curve(4.+dense/2)
        if np.any(left_dense<robot.limits[:7,0]) or np.any(left_dense>robot.limits[:7,1]):raise ValueError('left scan joint limit')
        if np.max(abs(curve.left_curve(4.+dense/2,1))/robot.velocity[:7])>.8:raise ValueError('left scan velocity limit')
        if np.max(abs(curve.left_curve(4.+dense/2,2))/robot.acceleration[:7])>.64:raise ValueError('left scan acceleration limit')
        full[:,:7]=curve.left_curve(t)
    # Prefix was already audited in the library; certify the changed suffix.
    margin=validate_conveyor_path(robot,full[t>=4.],canceled) if reuse_clearance is None else float(reuse_clearance)
    indices=np.r_[np.arange(7),np.arange(14,21)]
    checked=full[:,indices] if left_target is not None else q
    checked_indices=indices if left_target is not None else np.arange(14,21)
    dt=np.diff(t);vel=np.diff(checked,axis=0)/dt[:,None]
    acc=np.diff(np.vstack([np.zeros(checked.shape[1]),vel,np.zeros(checked.shape[1])]),axis=0)/np.r_[dt,dt[-1]][:,None]
    if reuse_clearance is None and (np.max(abs(vel)/robot.velocity[checked_indices])>1.001 or np.max(abs(acc)/robot.acceleration[checked_indices])>1.01):
        raise ValueError(f'sampled replay dynamics exceed hardware limits: velocity={np.max(abs(vel)/robot.velocity[checked_indices])}, acceleration={np.max(abs(acc)/robot.acceleration[checked_indices])}, min_dt={dt.min()}')
    plan=deepcopy(library.entries[key]) if reuse_clearance is None else copy(library.entries[key])
    plan['events']=dict(plan['events'])
    if reuse_clearance is None:plan.update(times_s=t.tolist(),positions_rad=q.tolist(),velocities_rad_s=curve(t,1).tolist())
    else:plan.update(times_s=t,positions_rad=q,velocities_rad_s=curve(t,1))
    if left_target is not None:
        positions=full[:,indices];velocities=np.c_[curve.left_curve(t,1),curve(t,1)]
        plan.update(joint_names=list(ARM_NAMES),positions_rad=positions.tolist() if reuse_clearance is None else positions,velocities_rad_s=velocities.tolist() if reuse_clearance is None else velocities)
    plan['events'].update(lift_end_s=4.+duration/2,stopped_s=4.+duration)
    info=dict(scan_completed=True,duration_s=4.+duration,waypoint_at_s=4.+duration/2,
        minimum_conveyor_clearance_m=margin,contact_s=3.,unchanged_prefix_until_s=4.)
    if left_target is not None:info.update(left_scan_parallel=True,left_scan_start_s=4.,left_scan_completed_s=4.+duration/2,left_scan_target_rad=np.asarray(left_target).tolist())
    return plan,curve,info


def compile_library(library,config,progress=None,left_target=None):
    accepted={};rejected={}
    for i,key in enumerate(library.entries):
        try:accepted[key]=compile_path(library,key,config,left_target=left_target)
        except ValueError as error:rejected[key]=str(error)
        if progress and (i+1)%100==0:progress(i+1,len(accepted),len(rejected))
    if not accepted:raise ValueError('no checked lift-scan replay available')
    return accepted,rejected


def restrict_selection(library,accepted):
    # Keep immutable original library data intact; only selectable views change.
    for name in ('normal_options','transverse_options'):
        options=getattr(library,name)
        setattr(library,name,{p:[k for k in keys if k in accepted] for p,keys in options.items() if any(k in accepted for k in keys)})
    library.normal_view.cell_options=library.normal_options
