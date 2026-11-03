"""Read-only measured-state approach followed by belt tracking and gradual alignment.
No ROS publishers/actions. Endpoint has nonzero velocity and cannot be sent as a
standalone trajectory without a continuously validated successor or braking path.
"""
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .model import RobotModel,pose,vector
from .conveyor import forecast
from .continuous_grasp_preview import jacobian
from .conveyor_screen import clearance
from .live_continuous_preview import roi_contains
_ROBOT=None

def warm(config):
 global _ROBOT
 _ROBOT=RobotModel(config['robot']);return True

def smooth(u):
 u=np.clip(u,0.,1.);return 10*u**3-15*u**4+6*u**5,30*u**2-60*u**3+30*u**4

def rotation_progress(angle,t,approach_s):
 """Smooth ramp to 15deg/s, then 30deg/s; smooth slowdown near target."""
 if angle<1e-12:return 0.,0.
 ramp=min(.3,approach_s/2);slow=np.radians(15.);fast=np.radians(30.)
 def integral(t):
  t=max(0.,t)
  if t>=ramp:return t-ramp/2,1.
  u=t/ramp;return ramp*(u**3-.5*u**4),3*u*u-2*u**3
 if t<=approach_s:
  area,speed=integral(t);budget=slow*area;rate=slow*speed
 else:
  tau=t-approach_s;area,speed=integral(tau)
  budget=slow*integral(approach_s)[0]+slow*tau+(fast-slow)*area
  rate=slow+(fast-slow)*speed
 brake=min(angle/2,np.radians(6.));begin=angle-brake
 if budget<=begin:return budget,rate
 u=np.clip((budget-begin)/(2*brake),0.,1.)
 return begin+2*brake*(u-u**3+.5*u**4),rate*(1-3*u*u+2*u**3)

def desired(initial,obj,v,grasp,height,approach_s,align_s,t,lookahead=1.,workspace=None,workspace_follow=True):
 """C2 position and orientation joins; 15deg/s during approach, 30deg/s afterwards."""
 first=forecast(obj,v,approach_s+lookahead)@grasp;first[2,3]+=height
 vb=np.array(v[:3],float)
 if workspace is not None:
  from .tracking_policy import constrained_point
  first[:3,3],vb=constrained_point(first[:3,3],vb,initial,workspace,workspace_follow)
 p0=initial[:3,3];p1=first[:3,3]
 target=forecast(obj,v,t+lookahead)@grasp;target[2,3]+=height
 if workspace is not None:target[:3,3],_=constrained_point(target[:3,3],vb,initial,workspace,workspace_follow)
 if t<=approach_s:
  u=t/approach_s;s,ds=smooth(u)
  # Quintic Hermite, initial v/a=0, terminal velocity=belt and acceleration=0.
  hv=-4*u**3+7*u**4-3*u**5;dhv=-12*u**2+28*u**3-15*u**4
  target[:3,3]=p0+s*(p1-p0)+hv*approach_s*vb
  linear=ds/approach_s*(p1-p0)+dhv*vb
 else:linear=vb
 rot=Rotation.from_matrix(target[:3,:3]@initial[:3,:3].T).as_rotvec();angle=float(np.linalg.norm(rot))
 progress,rate=rotation_progress(angle,t,approach_s);axis=rot/max(angle,1e-12)
 target[:3,:3]=Rotation.from_rotvec(progress*axis).as_matrix()@initial[:3,:3];angular=rate*axis
 return target,np.r_[linear,angular]

def bounded_joint_velocity(robot,q,wanted):
 """Slow before joint limits; infeasible incoming speeds still fail validation."""
 limits=robot.limits[14:21];a=.8*robot.acceleration[14:21];margin=.005+float(getattr(robot,"tracking_joint_reserve_rad",0.))
 lower=np.minimum(.98*robot.velocity[14:21],np.sqrt(2*a*np.maximum(q[14:21]-limits[:,0]-margin,0)))
 upper=np.minimum(.98*robot.velocity[14:21],np.sqrt(2*a*np.maximum(limits[:,1]-q[14:21]-margin,0)))
 return np.clip(wanted,-lower,upper)

def integrate(r,c,q,obj,v,grasp,approach_s,align_s,check_conveyor=True,dt=.02,tracking_window_s=None,residual_warning_only=False):
 q=vector(q,28).copy();initial=r.ee_fk('right',q);qd=np.zeros(28);total=approach_s+(align_s if tracking_window_s is None else min(align_s,tracking_window_s));times=np.linspace(0,total,int(np.ceil(total/dt))+1)
 path=[q.copy()];velocities=[qd.copy()];points=[];errors=[];angles=[];reason='';minimum=1e6
 def check(t):
  nonlocal minimum
  target,_=desired(initial,obj,v,grasp,c['approach_height'],approach_s,align_s,t,lookahead=float(c.get('tracking_lookahead_s',1.)),workspace=c.get('tracking_grasp_policy'),workspace_follow=c.get('_workspace_follow',True))
  fk=r.ee_fk('right',q);points.append(fk[:3,3].tolist());errors.append(float(np.linalg.norm(target[:3,3]-fk[:3,3])));angles.append(float(Rotation.from_matrix(target[:3,:3]@fk[:3,:3].T).magnitude()))
  r.clearance(q,c['obstacles'],c['collision_margin'])
  if check_conveyor:
   minimum=min(minimum,clearance(r,q))
   if minimum<0:raise ValueError('provisional conveyor collision')
  if not np.isfinite(errors[-1]) or not np.isfinite(angles[-1]):raise ValueError('nonfinite Cartesian residual')
  if not residual_warning_only and errors[-1]>r.ik_position_tolerance_m:raise ValueError('position tracking residual')
  if not residual_warning_only and angles[-1]>r.ik_rotation_tolerance_rad:raise ValueError('rotation tracking residual')
 try:
  check(0.)
  for a,b in zip(times[:-1],times[1:]):
   h=b-a;fk,j=jacobian(r,q);target,feed=desired(initial,obj,v,grasp,c['approach_height'],approach_s,align_s,a,lookahead=float(c.get('tracking_lookahead_s',1.)),workspace=c.get('tracking_grasp_policy'),workspace_follow=c.get('_workspace_follow',True))
   error=np.r_[target[:3,3]-fk[:3,3],Rotation.from_matrix(target[:3,:3]@fk[:3,:3].T).as_rotvec()]
   task=feed+np.r_[6*error[:3],4*error[3:]];weights=np.array([1,1,1,*([float(c.get('tracking_rotation_weight',.2))]*3)]);jw=j*weights[:,None]
   middle=r.limits[14:21].mean(1);span=np.diff(r.limits[14:21],axis=1).ravel();bias=.15*(middle-q[14:21])/span
   wanted=np.linalg.solve(jw.T@jw+.015**2*np.eye(7),jw.T@(weights*task)+.015**2*bias)
   if c.get('_initial_posture_reference') is not None:
    reference=np.asarray(c['_initial_posture_reference'],float)
    projector=np.eye(7)-np.linalg.solve(jw.T@jw+.015**2*np.eye(7),jw.T@jw)
    wanted += projector@np.clip(.5*(reference-q[14:21]),-.35,.35)
   wanted=bounded_joint_velocity(r,q,wanted) if residual_warning_only else np.clip(wanted,-r.velocity[14:21]*.98,r.velocity[14:21]*.98)
   next_qd=qd.copy();next_qd[14:21]+=np.clip(wanted-qd[14:21],-r.acceleration[14:21]*.95*h,r.acceleration[14:21]*.95*h)
   q=q+.5*(qd+next_qd)*h;qd=next_qd;path.append(q.copy());velocities.append(qd.copy());check(b)
 except ValueError as e:reason=str(e)
 ts=times[:len(path)];path=np.array(path);velocities=np.array(velocities);vr=ar=0.
 if len(path)>1:
  h=np.diff(ts);vv=np.diff(path,axis=0)/h[:,None]
  aa=np.vstack([(vv[0]-velocities[0])/(h[0]/2),np.diff(vv,axis=0)/((h[:-1]+h[1:])/2)[:,None],(velocities[-1]-vv[-1])/(h[-1]/2)])
  vr=float(np.max(abs(vv)/r.velocity));ar=float(np.max(abs(aa)/r.acceleration))
  if max(vr,ar)>1.00001:reason='independent joint dynamics check'
 return dict(valid=not reason,reason=reason,residual_warning_only=residual_warning_only,residual_exceeded=bool(max(errors,default=0.)>r.ik_position_tolerance_m or max(angles,default=0.)>r.ik_rotation_tolerance_rad),times=ts.tolist(),path=path.tolist(),ee_points=points,max_position_error_m=max(errors,default=0.),max_rotation_error_rad=max(angles,default=0.),velocity_ratio=vr,acceleration_ratio=ar,min_conveyor_margin_m=minimum,terminal_velocity=qd.tolist(),velocities=velocities.tolist())

def solve(config,q,object_pose,velocity,age_at_submit,submitted_monotonic,acquired_s,geometry):
 if _ROBOT is None:warm(config)
 began=time.monotonic();r=_ROBOT;q=vector(q,28);obj=pose(object_pose);v=vector(velocity,4)
 if np.linalg.norm(v[:3])>.3 or abs(v[3])>1e-8:raise ValueError('requires straight nonrotating belt motion')
 start_age=age_at_submit+began-submitted_monotonic;obj=forecast(obj,v,start_age);initial=r.ee_fk('right',q);base=pose(config['grasp_object_from_right_ee'])
 yaw=float(np.degrees(np.arctan2(obj[1,0],obj[0,0]))%180);chosen=float(np.clip(yaw,30,150));base[:3,:3]=Rotation.from_euler('z',chosen-yaw,degrees=True).as_matrix()@base[:3,:3]
 candidates=[]
 for flip in [0,180]:
  g=base.copy();g[:3,:3]=Rotation.from_euler('z',flip,degrees=True).as_matrix()@g[:3,:3]
  angle=float(Rotation.from_matrix((obj@g)[:3,:3]@initial[:3,:3].T).magnitude())
  # Quintic peak angular speed = 1.875*angle/T, cap at 30 deg/s.
  align=max(.6,1.875*angle/np.radians(30));candidates.append((angle,flip,g,align))
 candidates.sort(key=lambda x:x[0]);reports=[]
 for approach in [.8,1.2,1.6,2.]:
  for angle,flip,grasp,align in candidates:
   window=min(align,1.);total=approach+window
   meta=dict(flip=flip,approach_s=approach,align_s=align,tracking_window_s=window,angle_deg=float(np.degrees(angle)),observed_yaw_deg=yaw,grasp_yaw_deg=chosen)
   # Target has to remain in ROI through one-second lookahead at end of alignment.
   if not roi_contains(geometry,[forecast(obj,v,t)[:3,3] for t in np.linspace(0,total+1,12)]):reports.append(dict(meta,reason='ROI exit before short tracking segment ends'));continue
   result=integrate(r,config,q,obj,v,grasp,approach,align,tracking_window_s=window);reports.append(dict(meta,valid=result['valid'],reason=result['reason'],max_position_error_m=result['max_position_error_m']))
   elapsed=time.monotonic()-began
   if result['valid']:
    if elapsed>=approach:reports[-1]['reason']='planning consumed approach time';continue
    ts=np.array(result['times']);path=np.array(result['path']);idx=int(np.argmin(abs(ts-approach)));trace=[]
    for label,inds in [('Approach',np.flatnonzero(ts<=approach)),('Align',np.flatnonzero(ts>=approach))]:
     inds=inds[np.unique(np.linspace(0,len(inds)-1,min(30,len(inds)),dtype=int))];trace.append(dict(stage=label,times_s=(ts[inds]+start_age).tolist(),points=[result['ee_points'][i] for i in inds]))
    return dict(feasible=True,conditional=True,preview_only=True,preview_kind='position_first_alignment',actual_execution_feasible=False,actual_reason='measured-state upper approach + gradual alignment verified; no execution or descent',joint_start=q.tolist(),approach_pose=r.ee_fk('right',path[idx]).tolist(),alignment_pose=r.ee_fk('right',path[-1]).tolist(),object_pose=forecast(obj,v,total+1).tolist(),trace=trace,terminal_velocity=result['terminal_velocity'],requires_continuous_successor=True,acquired_s=acquired_s,contact_at_s=acquired_s+start_age+total,close_at_s=acquired_s+start_age+total,launch_at_s=acquired_s+start_age,prediction_horizon_s=start_age+total+1,durations_s=dict(Approach=approach,Align=window),alignment_complete=bool(angle-rotation_progress(angle,total,approach)[0]<np.radians(1)),rotation_remaining_deg=float(np.degrees(angle-rotation_progress(angle,total,approach)[0])),planning_ms=elapsed*1000,max_position_error_m=result['max_position_error_m'],max_rotation_error_rad=result['max_rotation_error_rad'],acceleration_ratio=result['acceleration_ratio'],velocity_ratio=result['velocity_ratio'],reports=reports,candidate_count=len(reports),feasible_count=1,method='position first; rotate gradually at <=15deg/s while approaching, then track and align at <=30deg/s; no descent/closure')
   if elapsed>2.5:break
  if time.monotonic()-began>2.5:break
 return dict(feasible=False,conditional=True,preview_kind='position_first_alignment',actual_execution_feasible=False,reason=reports[-1]['reason'] if reports else 'no candidate',reports=reports,planning_ms=(time.monotonic()-began)*1000,candidate_count=len(reports),feasible_count=0)
