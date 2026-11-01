"""Offline fixed-point build, no ROS publishers/action clients."""
import sys,json,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('tools').resolve()))
from fixed_interception import candidates,preflight
from preplanned_entry import correct
from reusable_intercept import signature,admit
root=Path('.setup/motion-alignment-audit/1793486182660462398')
old=json.loads((root/'motion_result.json').read_text())
current=json.loads(Path('.setup/moving-upper-trial/latest-intercept-input.json').read_text())
c=current['config'];p=json.loads(Path('calibration/tracking_grasp_policy.json').read_text());c['tracking_grasp_policy']=p
q=np.asarray(current['q']);c['_stationary_left_arm']=q[:7].tolist()
obj=np.array(old['object_at_start']);v=np.array(old['velocity']);grasp=np.array(old['grasp'])
# One previously feasible point/hand direction, frozen across bags.
obj[:3,3]-=v[:3]
choices=candidates(obj,v,grasp,old['approach_s'],c['approach_height'],p)
_,fixed=next((o,f) for o,f in choices if f['workspace_fraction']==.75)
began=time.perf_counter()
plan=correct(c,dict(old,fixed_interception=fixed),q,obj,v,0.,current['geometry'])
plan['fixed_descent_preflight']=preflight(c,plan)
for key in ('async_replan','success','reason','target_id','start_at','source_stamp','source_object','progressive_descent'):plan.pop(key,None)
record=dict(signature=signature(c),plan=plan,nominal_velocity=v.tolist(),built_at=time.time(),build_s=time.perf_counter()-began,
 source='offline replay of audited target 37; fixed point and grasp orientation; current ready joints',
 limits=dict(orientation_deg=15,height_m=.015,speed_delta_m_s=.03,start_joint_rad=.01))
out=Path('calibration/reusable_intercept.json');out.write_text(json.dumps(record,indent=2))
# Upstream evidence admits the same fixed path; deliberately include ample lead.
upstream=obj.copy();upstream[:3,3]-=v[:3]*2
began=time.perf_counter();r=admit(c,record,q,upstream,v,0.,current['geometry'])
print(json.dumps(dict(point=np.array(fixed['pose'])[:3,3].tolist(),build_s=record['build_s'],admission_ms=(time.perf_counter()-began)*1000,preview=plan['fixed_descent_preflight']),indent=2))
