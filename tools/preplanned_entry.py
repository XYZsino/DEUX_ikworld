"""Offline correction of a precomputed branch; never publishes robot commands."""
import copy,time
import numpy as np
from deux_task_core.conveyor import forecast
from deux_task_core.position_first_preview import integrate
from deux_task_core.live_continuous_preview import roi_contains
from deux_task_core.tracking_policy import follow_admitted
from right_only_trial import planning_robot
from intercept_window import add_brake
from async_upper import validate

def correct(config,template,q,obj,v,advance,geometry):
 began=time.perf_counter();c=copy.deepcopy(config);r=planning_robot(c)
 q=np.asarray(q);obj=forecast(np.asarray(obj),np.asarray(v),advance);v=np.asarray(v)
 grasp=np.asarray(template['grasp']);approach=template['approach_s']
 c['_workspace_follow']=follow_admitted(c['tracking_grasp_policy'],obj,v,grasp,1.)
 trial=template.get('initial_seed_trial',{})
 for seed in trial.get('seeds',[]):
  if seed['name']==trial.get('selected') and 'reference' in seed:c['_initial_posture_reference']=seed['reference']
 fixed=None;planning_obj=obj;planning_v=v
 if c['tracking_grasp_policy'].get('fixed_interception'):
  from fixed_interception import refresh
  fixed=refresh(template['fixed_interception'],obj,v,grasp,approach,c['approach_height'],c['tracking_grasp_policy'])
  planning_obj=np.asarray(fixed['pose'])@np.linalg.inv(grasp)
  planning_v=np.zeros(4);c['_workspace_follow']=True;c['tracking_lookahead_s']=0.
 upper=integrate(r,c,q,planning_obj,planning_v,grasp,approach,10.,tracking_window_s=.8,residual_warning_only=True,check_conveyor=False)
 if not upper['valid']:raise ValueError(upper['reason'])
 ts,path,vel,brake=add_brake(r,upper['times'],upper['path'],upper['velocities'])
 # Check both actual object and one-second-ahead target against the image ROI.
 if not roi_contains(geometry,[forecast(obj,v,t)[:3,3] for t in np.linspace(0,float(ts[-1])+(0. if fixed is not None else 1.),24)]):raise ValueError('corrected lookahead leaves ROI')
 out=dict(template,times=ts.tolist(),path=path.tolist(),velocities=vel.tolist(),initial_pose=r.ee_fk('right',q).tolist(),object_at_start=obj.tolist(),velocity=v.tolist(),geometry=geometry,workspace_follow=c['_workspace_follow'],brake_start_s=float(upper['times'][-1]),braking_s=brake,tracking_lookahead_s=1.)
 out['intercept']=dict(template.get('intercept',{}),object_position=forecast(obj,v,approach+1.)[:3,3].tolist())
 if fixed is not None:
  out['fixed_interception']=fixed;out['tracking_lookahead_s']=0.
  out['intercept']['object_position']=np.asarray(fixed['pose'])[:3,3].tolist()
 out['min_conveyor_margin_m']=validate(c,out,r)
 out['preplanned_correction_ms']=(time.perf_counter()-began)*1000
 return out
