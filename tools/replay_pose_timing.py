import sys,json,time
from pathlib import Path
import numpy as np
sys.path.insert(0,'tools')
from right_only_trial import planning_robot
from preplanned_entry import correct
from async_upper import compute_replacement
p=json.loads(Path('.setup/moving-upper-trial/latest-intercept-input.json').read_text());c=p['config'];policy=json.loads(Path('calibration/tracking_grasp_policy.json').read_text());c['tracking_grasp_policy']=policy;c['tracking_lookahead_s']=1.;c['approach_height']=.05;q=np.array(c['home_joints']);q[:7]=c['_stationary_left_arm'];q[14:21]=policy.get('standby_right_arm_rad',c['home_joints'][14:21]);r=planning_robot(c)
template=json.loads(Path('.setup/moving-upper-trial/result.json').read_text());g=np.array(template['grasp']);ee=r.ee_fk('right',q);contact=ee.copy();contact[2,3]-=.05;obj=contact@np.linalg.inv(g);v=np.array([0.,-.03,0.,0.])
old=correct(c,template,q,obj,v,0.,p['geometry']);old.update(start_at=0.,target_id='offline-scheduled',source_object=obj.tolist(),source_stamp=0.)
c['_descent_admitted']=True;c['_descent_measured_height']=.05
rows=[]
for splice in [.6,1.0,1.4,1.8,2.2]:
 t=time.perf_counter();old=compute_replacement(c,old,obj,v,0.,splice,len(rows)+1)
 rows.append(dict(splice=splice,compute_s=time.perf_counter()-t,profile=old['descent_profile'],clearance=old['min_conveyor_margin_m'],timing=old.get('descent_timing')))
print(rows)
Path('.setup/pose-timing-replay.json').write_text(json.dumps(dict(synthetic=True,rows=rows),indent=2))
