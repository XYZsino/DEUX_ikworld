import sys,json,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('tools').resolve()))
from preplanned_entry import correct
from async_upper import compute_replacement
root=Path('.setup/motion-alignment-audit/1793486182660462398')
c=json.loads((root/'config.json').read_text());old=json.loads((root/'motion_result.json').read_text())
c['tracking_grasp_policy']=json.loads(Path('calibration/tracking_grasp_policy.json').read_text());c['tracking_grasp_policy']['fixed_interception']=True
c['approach_height']=.05;c['tracking_rotation_weight']=.05;c['progressive_descent']=True
c['intercept_window']=json.loads(Path('calibration/intercept_window.json').read_text());c['upper_replan_horizon_s']=.8
q=np.array(old['path'][0]);c['_stationary_left_arm']=q[:7].tolist()
obj=np.array(old['object_at_start']);v=np.array(old['velocity']);results=[]
for earlier in [0.,.5,1.]:
 try:
  source=obj.copy();source[:3,3]-=v[:3]*earlier
  began=time.perf_counter();plan=correct(c,old,q,source,v,0.,old['geometry'])
  plan.update(start_at=100.,target_id='offline',generation=0)
  newer=source.copy();newer[:3,3]+=v[:3]*.5
  replacement=compute_replacement(c,plan,newer,v,100.5,.7,1)
  assert replacement['fixed_interception']==plan['fixed_interception']
  results.append(dict(earlier_s=earlier,valid=True,elapsed_s=time.perf_counter()-began,fixed=plan['fixed_interception'],margin=replacement['min_conveyor_margin_m']))
 except ValueError as e:results.append(dict(earlier_s=earlier,valid=False,reason=str(e)))
Path('.setup/fixed-interception-backup/replay.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
