"""Offline differential-IK tracking/descent/closure validation. Not an action backend.

Carries incoming joint velocity and leaves terminal velocity nonzero: a real
executor would need a continuous successor, not append an implicit stop.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from .conveyor import forecast
from .model import pose,vector

def jacobian(robot,q):
    original=robot.ee_fk('right',q);j=np.zeros((6,7));eps=1e-5
    for k,index in enumerate(range(14,21)):
        shifted=q.copy();shifted[index]+=eps;fk=robot.ee_fk('right',shifted)
        j[:3,k]=(fk[:3,3]-original[:3,3])/eps
        j[3:,k]=Rotation.from_matrix(fk[:3,:3]@original[:3,:3].T).as_rotvec()/eps
    return original,j

def plan(planner,start,start_velocity,object_pose,velocity,descent_s,close_s=.5,dt=.02,residual_warning_only=False,bound_joints=False):
    r=planner.robot;c=planner.config;q=vector(start,28).copy();qd=vector(start_velocity,28).copy();obj=pose(object_pose);v=vector(velocity,4)
    if not 0<descent_s<=4 or close_s!=.5 or not 0<dt<=.02:raise ValueError('invalid preview timing')
    if np.any(np.abs(qd)>r.velocity):raise ValueError('incoming velocity outside limits')
    if np.any(abs(qd[:14])>1e-8) or np.any(abs(qd[21:])>1e-8):raise ValueError('preview supports right arm motion only')
    if np.linalg.norm(v[:3])>.3 or abs(v[3])>1e-8:raise ValueError('preview requires straight nonrotating belt motion')
    grasp=pose(c['grasp_object_from_right_ee']);initial=r.ee_fk('right',q);target0=obj@grasp;offset=initial[:3,3]-target0[:3,3]
    rot_offset=Rotation.from_matrix(initial[:3,:3]@target0[:3,:3].T).as_rotvec()
    total=descent_s+close_s;times=np.linspace(0,total,int(np.ceil(total/dt))+1);path=[q.copy()];velocities=[qd.copy()];positions=[];desired_points=[];errors=[];rot_errors=[];minimum=float('inf');reason=''
    def desired(t):
        u=min(1.,t/descent_s);s=10*u**3-15*u**4+6*u**5;ds=(30*u*u-60*u**3+30*u**4)/descent_s if t<descent_s else 0.
        target=forecast(obj,v,t)@grasp;target[:3,3]+=(1-s)*offset;target[:3,:3]=Rotation.from_rotvec((1-s)*rot_offset).as_matrix()@target[:3,:3]
        return target,np.r_[v[:3]-ds*offset,-ds*rot_offset]
    def check(t,state):
        nonlocal minimum
        target,_=desired(t);fk=r.ee_fk('right',state)
        errors.append(float(np.linalg.norm(fk[:3,3]-target[:3,3])));rot_errors.append(float(Rotation.from_matrix(target[:3,:3]@fk[:3,:3].T).magnitude()))
        positions.append(fk[:3,3].tolist());desired_points.append(target[:3,3].tolist())
        minimum=min(minimum,r.clearance(state,c['obstacles'],c['collision_margin'],0.))
    try:
        check(0,q)
        for a,b in zip(times[:-1],times[1:]):
            h=b-a;fk,j=jacobian(r,q);target,feed=desired(a)
            error=np.r_[target[:3,3]-fk[:3,3],Rotation.from_matrix(target[:3,:3]@fk[:3,:3].T).as_rotvec()]
            task=feed+np.r_[6.*error[:3],4.*error[3:]]
            weights=np.array([1,1,1,*([float(c.get('tracking_rotation_weight',.2))]*3)]);jw=j*weights[:,None]
            midpoint=r.limits[14:21].mean(axis=1);span=np.diff(r.limits[14:21],axis=1).ravel()
            bias=.15*(midpoint-q[14:21])/span
            wanted=np.linalg.solve(jw.T@jw+.015**2*np.eye(7),jw.T@(weights*task)+.015**2*bias)
            if bound_joints:
                from .position_first_preview import bounded_joint_velocity
                wanted=bounded_joint_velocity(r,q,wanted)
            else:wanted=np.clip(wanted,-r.velocity[14:21]*.98,r.velocity[14:21]*.98)
            new_qd=qd.copy();new_qd[14:21]+=np.clip(wanted-qd[14:21],-r.acceleration[14:21]*.95*h,r.acceleration[14:21]*.95*h)
            q=q+.5*(qd+new_qd)*h;qd=new_qd;path.append(q.copy());velocities.append(qd.copy());check(b,q)
        if not residual_warning_only and max(errors)>r.ik_position_tolerance_m:reason='position tracking residual'
        elif not residual_warning_only and max(rot_errors)>r.ik_rotation_tolerance_rad:reason='rotation tracking residual'
    except ValueError as e:reason=str(e)
    ts=times[:len(path)];path=np.asarray(path);velocities=np.asarray(velocities)
    # Independent finite-difference limits, using incoming velocity (never zero).
    vv=np.diff(path,axis=0)/np.diff(ts)[:,None]
    aa=np.diff(velocities,axis=0)/np.diff(ts)[:,None]
    if len(vv):
        intervals=np.diff(ts)
        independent_accel=np.vstack([(vv[0]-velocities[0])/(intervals[0]/2),np.diff(vv,axis=0)/((intervals[:-1]+intervals[1:])/2)[:,None],(velocities[-1]-vv[-1])/(intervals[-1]/2)])
        aa=np.vstack([aa,independent_accel])
    vr=float(np.max(np.abs(vv)/r.velocity)) if len(vv) else 0.;ar=float(np.max(np.abs(aa)/r.acceleration)) if len(aa) else 0.
    if max(vr,ar)>1+1e-8:reason='independent joint dynamics check'
    return dict(valid=not reason,reason=reason,times=ts.tolist(),path=path.tolist(),velocities=velocities.tolist(),ee_points=positions,desired_points=desired_points,max_position_error_m=max(errors,default=0.),max_rotation_error_rad=max(rot_errors,default=0.),velocity_ratio=vr,acceleration_ratio=ar,minimum_clearance=minimum,descent_s=descent_s,close_s=close_s,incoming_velocity=start_velocity.tolist() if hasattr(start_velocity,'tolist') else start_velocity,terminal_velocity=velocities[-1].tolist(),requires_continuous_successor=True)
