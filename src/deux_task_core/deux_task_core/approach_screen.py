"""Read-only current-joint to frozen-upper-target screening; not an intercept plan."""
import numpy as np
from .conveyor_screen import clearance

def screen(robot,config,current,target,velocity):
 current=np.asarray(current,float);target=np.asarray(target,float).copy()
 # Right-arm-only command; other joints stay at measured/reference values.
 target[:14]=current[:14];target[21:]=current[21:]
 delta=abs(target[14:21]-current[14:21]);v=np.minimum(robot.velocity[14:21],.25);a=np.minimum(robot.acceleration[14:21],.3)
 duration=max(.5,float(np.max(1.5*delta/v)),float(np.sqrt(np.max(6*delta/a))))+.2
 t=np.linspace(0,duration,int(np.ceil(duration/.02))+1);u=t/duration
 path=current+(3*u*u-2*u*u*u)[:,None]*(target-current)
 out=dict(kind='frozen upper target geometry only',passed=False,duration_s=duration,target_drift_during_motion_m=float(np.linalg.norm(np.asarray(velocity)[:3])*duration),actual_execution_feasible=False,reason='',start=current.tolist(),goal=target.tolist())
 limit=robot.limits
 if np.any(path<limit[:,0]-robot.joint_limit_tolerance) or np.any(path>limit[:,1]+robot.joint_limit_tolerance):out['reason']='approach joint limit';return out
 vel=np.diff(path,axis=0)/np.diff(t)[:,None];acc=np.diff(np.vstack([np.zeros(28),vel,np.zeros(28)]),axis=0)/np.r_[np.diff(t),np.diff(t)[-1]][:,None]
 if np.any(abs(vel)>robot.velocity*1.001) or np.any(abs(acc)>robot.acceleration*1.01):out['reason']='approach velocity/acceleration limit';return out
 minimum=float('inf')
 for row in path:
  try:robot.clearance(row,config['obstacles'],config['collision_margin'])
  except ValueError as e:out['reason']='configured collision/limit: '+str(e);return out
  minimum=min(minimum,clearance(robot,row))
  if minimum<0:out.update(reason='approach intersects provisional conveyor envelope',min_conveyor_margin_m=minimum);return out
 out.update(passed=True,reason='frozen upper path passed; moving-target arrival not verified',min_conveyor_margin_m=minimum)
 return out
