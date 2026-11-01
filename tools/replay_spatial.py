import sys,json,time
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('tools').resolve()))
from fixed_interception import candidates,preflight
from preplanned_entry import correct
root=Path('.setup/motion-alignment-audit/1793486182660462398')
c=json.loads((root/'config.json').read_text());old=json.loads((root/'motion_result.json').read_text())
c['tracking_grasp_policy']=json.loads(Path('calibration/tracking_grasp_policy.json').read_text())
c.update(approach_height=.05,tracking_rotation_weight=.05,progressive_descent=True,upper_replan_horizon_s=.8)
c['intercept_window']=json.loads(Path('calibration/intercept_window.json').read_text())
q=np.array(old['path'][0]);c['_stationary_left_arm']=q[:7].tolist()
obj=np.array(old['object_at_start']);v=np.array(old['velocity']);g=np.array(old['grasp']);results=[]
for earlier in [1.,2.]:
 source=obj.copy();source[:3,3]-=v[:3]*earlier
 try:choices=candidates(source,v,g,old['approach_s'],.05,c['tracking_grasp_policy'])
 except ValueError as e:results.append(dict(earlier=earlier,error=str(e)));continue
 for _,fixed in choices:
  began=time.perf_counter()
  try:
   plan=correct(c,dict(old,fixed_interception=fixed),q,source,v,0.,old['geometry'])
   result=preflight(c,plan)
   results.append(dict(earlier=earlier,fraction=fixed['workspace_fraction'],valid=True,seconds=time.perf_counter()-began,result=result))
  except ValueError as e:results.append(dict(earlier=earlier,fraction=fixed['workspace_fraction'],valid=False,seconds=time.perf_counter()-began,error=str(e)))
print(json.dumps(results,indent=2));Path('.setup/fixed-spatial-backup/replay.json').write_text(json.dumps(results,indent=2))
