import json,copy
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from deux_task_core.configuration import load_configuration
from deux_task_core.planning import Planner
from deux_task_core.model import JOINT_NAMES
root=Path('/home/xyz_thor/DEUX_ikworld');out=root/'.setup/ready-30';out.mkdir(exist_ok=True)
c,_=load_configuration(str(root/'src/deux_task_bringup/config/installation.yaml'));c=copy.deepcopy(c);c['robot']['ik_position_tolerance_m']=.005;c['robot']['ik_rotation_tolerance_rad']=np.radians(3)
r=Planner(c).robot
old=json.loads((root/'.setup/roi-ready/candidate.json').read_text());base=np.array(old['chosen']['new_home']);target=r.ee_fk('right',base);target[:3,:3]=Rotation.from_euler('z',30,degrees=True).as_matrix()@target[:3,:3]
raw=(root/'.setup/ready-arm-trial/current-joints.yaml').read_text();s=next(yaml.safe_load_all('header:'+raw.split('header:',1)[1]));current=base.copy()
for k,v in zip(s['name'],s['position']):current[JOINT_NAMES.index(k)]=v
print('current_right_deg',np.degrees(current[14:21]).tolist(),flush=True)
results=[]
for seed in [base,current,np.array(c['home_joints'])]:
 try:
  q=r.ik('right',target,seed,continuity_weight=.002);actual=r.ee_fk('right',q);r.clearance(q,c['obstacles'],c['collision_margin']);margin=np.min(np.minimum(q[14:21]-r.limits[14:21,0],r.limits[14:21,1]-q[14:21]));results.append(dict(q=q.tolist(),actual=actual.tolist(),margin_deg=float(np.degrees(margin)),position_error_m=float(np.linalg.norm(actual[:3,3]-target[:3,3])),rotation_error_deg=float(np.degrees(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude()))))
 except Exception as e:print(type(e).__name__,str(e),flush=True)
if not results:
 from scipy.optimize import least_squares
 rng=np.random.default_rng(30);bounds=r.limits[14:21];lo=bounds[:,0]+.02;hi=bounds[:,1]-.02
 def residual(arm):
  q=base.copy();q[14:21]=arm;actual=r.ee_fk('right',q)
  return np.r_[(actual[:3,3]-target[:3,3])*5,Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).as_rotvec()]
 for i in range(32):
  seed=base[14:21]+rng.normal(0,.7 if i<16 else 1.8,7)
  fit=least_squares(residual,np.clip(seed,lo+1e-5,hi-1e-5),bounds=(lo,hi),max_nfev=160)
  err=residual(fit.x)
  if np.linalg.norm(err[:3])/5>.005 or np.linalg.norm(err[3:])>np.radians(3):continue
  q=base.copy();q[14:21]=fit.x;actual=r.ee_fk('right',q)
  try:r.clearance(q,c['obstacles'],c['collision_margin'])
  except ValueError:continue
  results.append(dict(q=q.tolist(),actual=actual.tolist(),margin_deg=float(np.degrees(np.min(np.minimum(fit.x-bounds[:,0],bounds[:,1]-fit.x)))),position_error_m=float(np.linalg.norm(err[:3])/5),rotation_error_deg=float(np.degrees(np.linalg.norm(err[3:]))),solver='scipy bounded URDF FK least squares'))
  if len(results)>=4:break
if not results:raise RuntimeError('no ready30 solution including multiseed')
results.sort(key=lambda x:np.linalg.norm(np.array(x['q'])[14:21]-base[14:21]));best=results[0];new=base.copy();new[14:21]=np.array(best['q'])[14:21]
d=dict(old_home=base.tolist(),current=current.tolist(),target=target.tolist(),chosen=dict(new_home=new.tolist(),**best),results=results,object_yaw_deg=30)
(out/'candidate.json').write_text(json.dumps(d,indent=2));print('chosen',best,flush=True)
