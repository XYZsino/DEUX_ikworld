"""Mandatory robot-right (-base Y) Cartesian escape before upper approach.
No ROS, action clients or commands. Never falls back to a direct shortcut.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from right_only_trial import planning_robot
from async_upper import validate,spline
from deux_task_core.conveyor_screen import surfaces,WRIST_GRASP_RADIUS_M,EXTRA_MARGIN_M


def waypoints(initial,target,extra=.03):
    if not np.isfinite(extra) or extra<.03 or extra>.15:raise ValueError('invalid right corridor clearance')
    # Only measured surfaces overlapping the approach X span determine the side corridor.
    allowance=WRIST_GRASP_RADIUS_M+EXTRA_MARGIN_M+surfaces()[4]+extra
    xlo=min(initial[0,3],target[0,3])-allowance;xhi=max(initial[0,3],target[0,3])+allowance
    polys=[vertices for vertices,_,_ in surfaces()[3] if vertices[:,0].min()<=xhi and vertices[:,0].max()>=xlo]
    if not polys:raise ValueError('no measured conveyor surface for right corridor')
    right_y=min(float(initial[1,3]),float(min(p[:,1].min() for p in polys)-allowance))
    clear_z=max(float(initial[2,3]),float(target[2,3]))
    out=[];right=np.array(initial).copy();right[1,3]=right_y;out.append(('RightEscape',right))
    raised=right.copy();raised[2,3]=clear_z;out.append(('RaiseOutsideConveyor',raised))
    forward=np.array(target).copy();forward[1,3]=right_y;forward[2,3]=clear_z;out.append(('AdvanceOnRight',forward))
    out.append(('EnterUpperWait',np.array(target).copy()))
    return out,dict(direction='base_link -Y (robot right)',right_corridor_y_m=right_y,extra_clearance_m=extra,
                    surface_allowance_m=allowance,upper_target_xyz_m=target[:3,3].tolist())


def interpolate(a,b,u):
    s=10*u**3-15*u**4+6*u**5
    pose=a.copy();pose[:3,3]=a[:3,3]+s*(b[:3,3]-a[:3,3])
    rot=Rotation.from_matrix(b[:3,:3]@a[:3,:3].T).as_rotvec()
    pose[:3,:3]=Rotation.from_rotvec(s*rot).as_matrix()@a[:3,:3]
    return pose


def segment(config,r,q,a,b,name):
    distance=float(np.linalg.norm(b[:3,3]-a[:3,3]));angle=Rotation.from_matrix(b[:3,:3]@a[:3,:3].T).magnitude()
    duration=max(1.,1.875*distance/.08,1.875*angle/np.deg2rad(15))
    n=max(20,int(np.ceil(distance/.004)),int(np.ceil(angle/np.deg2rad(2))))
    u=np.linspace(0,1,n+1);path=[np.array(q).copy()]
    for value in u[1:]:
        desired=interpolate(a,b,value)
        seed=q.copy();reserve=np.deg2rad(float(config.get('intercept_window',{}).get('min_nominal_joint_margin_deg',0.)))+.003
        lo=r.limits[14:21,0]+reserve;hi=r.limits[14:21,1]-reserve
        orientation_weight=.01 if name in ('RightEscape','RaiseOutsideConveyor') else .15
        def residual(arm):
            row=seed.copy();row[14:21]=arm;fk=r.ee_fk('right',row)
            return np.r_[fk[:3,3]-desired[:3,3],orientation_weight*Rotation.from_matrix(desired[:3,:3]@fk[:3,:3].T).as_rotvec(),.001*(arm-seed[14:21])]
        solved=least_squares(residual,np.clip(seed[14:21],lo+1e-8,hi-1e-8),bounds=(lo,hi),max_nfev=180,ftol=1e-10,gtol=1e-10,xtol=1e-10)
        q=seed.copy();q[14:21]=solved.x
        if np.linalg.norm(r.ee_fk('right',q)[:3,3]-desired[:3,3])>.003:raise ValueError(name+': Cartesian IK residual '+str(np.linalg.norm(r.ee_fk('right',q)[:3,3]-desired[:3,3]))+' target='+str(desired[:3,3]))
        path.append(q.copy())
    path=np.array(path)
    for attempt in range(12):
        times=u*duration;vel=np.gradient(path,times,axis=0,edge_order=2);vel[[0,-1]]=0.
        plan=dict(times=times.tolist(),path=path.tolist(),velocities=vel.tolist())
        try:
            clearance=validate(config,plan,r)
            break
        except ValueError as error:
            if 'continuous polynomial limit order=1' not in str(error) and 'continuous polynomial limit order=2' not in str(error):raise
            duration*=1.3
    else:raise ValueError(name+': joint retiming did not converge')
    # Check the actual joint spline follows the mandated Cartesian segment.
    curve=spline(plan);worst=0.;positions=[]
    for t in np.linspace(0,duration,int(np.ceil(duration/.02))+1):
        actual=r.ee_fk('right',curve(t));wanted=interpolate(a,b,t/duration)
        error=float(np.linalg.norm(actual[:3,3]-wanted[:3,3]));worst=max(worst,error);positions.append(actual[:3,3])
        if error>.005:raise ValueError(name+': straight-segment tracking residual')
    return plan,dict(name=name,duration_s=duration,start_xyz_m=a[:3,3].tolist(),end_xyz_m=b[:3,3].tolist(),
                     max_cartesian_residual_m=worst,min_conveyor_margin_m=clearance)


def plan_right_approach(config,current,target):
    r=planning_robot(config);q=np.array(current).copy();initial=r.ee_fk('right',q)
    points,metadata=waypoints(initial,target,float(config.get('right_approach_extra_clearance_m',.03)))
    out=dict(times=[0.],path=[q.tolist()],velocities=[np.zeros(28).tolist()]);stages=[];a=initial
    for name,b in points:
        distance=np.linalg.norm(b[:3,3]-a[:3,3]);angle=Rotation.from_matrix(b[:3,:3]@a[:3,:3].T).magnitude()
        if distance<1e-4 and angle<1e-3:continue
        part,record=segment(config,r,q,a,b,name);offset=out['times'][-1]
        out['times'] += (np.array(part['times'][1:])+offset).tolist()
        out['path'] += part['path'][1:];out['velocities'] += part['velocities'][1:]
        record.update(start_s=offset,end_s=out['times'][-1]);stages.append(record)
        q=np.array(part['path'][-1]);a=r.ee_fk('right',q)
    if len(out['times'])<2:
        out.update(times=[0.,.5],path=[q.tolist(),q.tolist()],velocities=[np.zeros(28).tolist()]*2)
    out['min_conveyor_margin_m']=validate(config,out,r)
    out['right_approach']=dict(metadata,stages=stages,policy='mandatory right escape, outside lift, right-side advance, upper entry; no direct fallback')
    return out
