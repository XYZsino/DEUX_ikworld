import sys,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,'tools')
from initial_seed_trial import backend_robot
from deux_task_core.conveyor_screen import clearance
from async_upper import validate
p=json.loads(Path('.setup/moving-upper-trial/latest-intercept-input.json').read_text());c=p['config'];r=backend_robot(c)
q=np.array(c['home_joints']);base=r.ee_fk('right',q)
seeds=[q.copy()]
for arm in c['tracking_grasp_policy']['demo_right_arm_rad']:
 s=q.copy();s[14:21]=arm;seeds.append(s)
results=[];fail=[]
for yaw in [-15.,0.,15.]:
 target=base.copy();target[:3,3]=[.59,-.08,.98];target[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix()@base[:3,:3]
 for index,seed in enumerate(seeds):
  try:
   solved=r.ik('right',target,seed);path=[solved.copy()]
   if np.linalg.norm(r.ee_fk('right',solved)[:3,3]-target[:3,3])>.01:raise ValueError('standby position residual')
   for u in np.linspace(0,1,21)[1:]:
    dest=target.copy();dest[1,3]-=.05*u;dest[2,3]-=.08*min(1,u/.8)
    solved=r.ik('right',dest,solved)
    if np.linalg.norm(r.ee_fk('right',solved)[:3,3]-dest[:3,3])>.01:raise ValueError('path residual')
    path.append(solved.copy())
   path=np.array(path);lo=r.limits[14:21,0];hi=r.limits[14:21,1]
   reserve=np.min(np.minimum(path[:,14:21]-lo,hi-path[:,14:21]));j6=np.min(np.minimum(path[:,19]-lo[5],hi[5]-path[:,19]))
   if reserve<np.deg2rad(5) or j6<np.deg2rad(12):raise ValueError('reserve')
   for duration in [3.,4.,6.]:
    times=np.linspace(0,duration,len(path));vel=np.gradient(path,times,axis=0);vel[0]=0;vel[-1]=0
    plan=dict(times=times.tolist(),path=path.tolist(),velocities=vel.tolist())
    try:margin=validate(c,plan,r);break
    except ValueError:
     if duration==6.:raise
   results.append(dict(yaw=yaw,seed=index,right_arm_rad=path[0,14:21].tolist(),ee= r.ee_fk('right',path[0]).tolist(),min_reserve_deg=float(np.degrees(reserve)),min_j6_reserve_deg=float(np.degrees(j6)),clearance_m=margin,duration=duration,validation_path=plan))
  except ValueError as e:fail.append(dict(yaw=yaw,seed=index,reason=str(e)))
results.sort(key=lambda x:(x['min_j6_reserve_deg'],x['min_reserve_deg']),reverse=True)
out=dict(reference='synthetic 8cm descent plus 5cm belt follow; not hardware verification',candidates=results,failures=fail,original_ready_fk=base.tolist())
Path('.setup/scheduled-standby-candidates.json').write_text(json.dumps(out,indent=2));print(json.dumps(dict(valid=len(results),best={k:v for k,v in results[0].items() if k not in ['validation_path','ee']} if results else None,failures=fail[:3])))
