"""Read-only interception candidates. No command publisher or executable plan output."""
from copy import deepcopy
from types import SimpleNamespace
import time
import numpy as np
from scipy.spatial.transform import Rotation
from .planning import Planner
from .conveyor import plan_conveyor,forecast
from .model import pose
_CACHE=None

def warm(config):
 global _CACHE
 _CACHE=Planner(config);q=np.array(config['home_joints']);_CACHE.robot.ik('right',_CACHE.robot.ee_fk('right',q),q)
 return True

def solve(config,q,object_pose,velocity,age_at_submit,submitted_monotonic,acquired_s):
 global _CACHE
 if _CACHE is None:warm(config)
 began=time.monotonic();r=_CACHE.robot;q=np.asarray(q);obj=pose(object_pose);v=np.asarray(velocity)
 base=deepcopy(config)
 # Full descent is preview-only; preserve actual BT's fixed-distance descent config.
 base['conveyor_tracking']=dict(workspace_policy='ik',max_speed=.3,start_lead=.5,max_plan_horizon=9.,intercept_wait_s=0.,durations={'Approach':1.5,'Descend':1.,'Close':config['hand_close_duration_s']})
 # Real hand is a scalar actuator. Plan arm following, not fictitious finger splines.
 held=np.r_[q[7:14],q[21:28]].tolist();base['hand_pregrasp']=held;base['hand_closed']=held
 transform=pose(base['grasp_object_from_right_ee']);reference=r.ee_fk('right',np.array(config['home_joints']))[:3,:3]
 ordered=sorted((float(Rotation.from_matrix(reference.T@obj[:3,:3]@Rotation.from_euler('z',yaw,degrees=True).as_matrix()@transform[:3,:3]).magnitude()),yaw) for yaw in (0,180))
 # Fixed launch instant across candidates; measured computation must fit this allowance.
 start_lead=age_at_submit+(time.monotonic()-submitted_monotonic)+1.5
 reports=[];feasible=[]
 for duration in (.9,1.5,2.2,3.):
  for _,yaw in ordered:
   c=deepcopy(base);grasp=transform.copy();grasp[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@transform[:3,:3];c['grasp_object_from_right_ee']=grasp.tolist()
   c['conveyor_tracking']['durations']['Approach']=duration
   trial=SimpleNamespace(config=c,robot=r);lead=start_lead;plans=[];seed=q;stage='Approach';report=dict(yaw_deg=yaw,requested_approach_s=duration)
   try:
    for stage in ('Approach','Descend','Close'):
     result=plan_conveyor(trial,stage,seed,obj,v,lead)
     plans.append((stage,lead,result));lead+=float(result['times'][-1]);seed=result['path'][-1]
     if lead-age_at_submit>7.:raise ValueError('closing completion beyond 7 second preview horizon')
    path=np.vstack([result['path'] for _,_,result in plans]);arm=path[:,14:21]
    margin=float(np.min(np.minimum(arm-r.limits[14:21,0],r.limits[14:21,1]-arm)))
    travel=float(np.linalg.norm(np.diff(arm,axis=0),axis=1).sum())
    contact_lead=plans[2][1];close_lead=lead
    contact=r.ee_fk('right',plans[1][2]['path'][-1]);closed=r.ee_fk('right',plans[2][2]['path'][-1])
    traces=[]
    for stage,phase_start,result in plans:
     indices=np.unique(np.r_[np.arange(0,len(result['times']),max(1,len(result['times'])//16)),len(result['times'])-1])
     traces.append(dict(stage=stage,times_s=[float(phase_start+result['times'][i]) for i in indices],points=[r.ee_fk('right',result['path'][i])[:3,3].tolist() for i in indices]))
    score=travel+.2*(close_lead-start_lead)+.03/(max(0.,margin)+.02)
    report.update(feasible=True,contact_at_s=acquired_s+contact_lead,close_at_s=acquired_s+close_lead,margin_deg=float(np.degrees(margin)),score=score)
    feasible.append(dict(feasible=True,yaw_deg=yaw,acquired_s=acquired_s,contact_at_s=acquired_s+contact_lead,close_at_s=acquired_s+close_lead,launch_at_s=acquired_s+start_lead,grasp_pose=contact.tolist(),close_pose=closed.tolist(),nominal_grasp_pose=(forecast(obj,v,contact_lead)@grasp).tolist(),object_pose=forecast(obj,v,contact_lead).tolist(),close_object_pose=forecast(obj,v,close_lead).tolist(),approach_pose=r.ee_fk('right',plans[0][2]['path'][-1]).tolist(),durations_s={stage:float(result['times'][-1]) for stage,_,result in plans},trace=traces,margin_deg=report['margin_deg'],score=score,joint_start=q.tolist(),prediction_horizon_s=contact_lead,remaining_height_m=0.,preview_full_descent=True,actual_bt_descend_distance_m=float(config['descend_distance_m']),hand_timing_only=True,method='candidate interception preview; arm follows during scalar closure; no execution'))
   except ValueError as e:
    cause=e
    while cause.__cause__ is not None:cause=cause.__cause__
    report.update(feasible=False,failed_stage=stage,reason=str(cause))
   reports.append(report)
   # Stop exhaustive search once the display schedule is about to become obsolete.
   if time.monotonic()-submitted_monotonic>=start_lead-age_at_submit-.2:break
  else:continue
  break
 elapsed=(time.monotonic()-began)*1000
 if feasible and start_lead-(age_at_submit+time.monotonic()-submitted_monotonic)>.15:
  result=min(feasible,key=lambda x:x['score']);result.update(reports=reports,planning_ms=elapsed,candidate_count=len(reports),feasible_count=len(feasible));return result
 return dict(feasible=False,reason='candidate schedule expired during calculation' if feasible else (reports[-1]['reason'] if reports else 'no candidates'),reports=reports,planning_ms=elapsed,candidate_count=len(reports),feasible_count=len(feasible))
