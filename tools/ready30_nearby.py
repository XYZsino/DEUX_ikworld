import json,time
from pathlib import Path
import numpy as np,yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from deux_task_core.configuration import load_configuration
from deux_task_core.model import RobotModel,JOINT_NAMES
from ready_conveyor_geometry import clearance
root=Path('/home/xyz_thor/DEUX_ikworld');out=root/'.setup/ready-30';c,_=load_configuration(str(root/'src/deux_task_bringup/config/installation.yaml'));r=RobotModel(c['robot'])
old=np.array(json.loads((root/'.setup/roi-ready/candidate.json').read_text())['chosen']['new_home']);target=r.ee_fk('right',old);target[:3,:3]=Rotation.from_euler('z',30,degrees=True).as_matrix()@target[:3,:3]
raw=(root/'.setup/ready-arm-trial/current-joints.yaml').read_text();s=next(yaml.safe_load_all('header:'+raw.split('header:',1)[1]));current=old.copy()
for k,v in zip(s['name'],s['position']):current[JOINT_NAMES.index(k)]=v
bounds=r.limits[14:21];lo=bounds[:,0]+np.radians(5);hi=bounds[:,1]-np.radians(5);rng=np.random.default_rng(30);results=[]
def residual(arm):
 q=current.copy();q[14:21]=arm;actual=r.ee_fk('right',q)
 return np.r_[(actual[:3,3]-target[:3,3])*2,Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).as_rotvec()*30]
for i in range(40):
 seed=old[14:21]+rng.normal(0,.7 if i<20 else 1.8,7)
 fit=least_squares(residual,np.clip(seed,lo+1e-6,hi-1e-6),bounds=(lo,hi),max_nfev=180)
 q=current.copy();q[14:21]=fit.x;actual=r.ee_fk('right',q);dp=actual[:3,3]-target[:3,3];angle=np.degrees(Rotation.from_matrix(target[:3,:3].T@actual[:3,:3]).magnitude())
 if angle>1 or np.linalg.norm(dp)>.2 or clearance(r,q)<0:continue
 try:r.clearance(q,c['obstacles'],c['collision_margin'])
 except ValueError:continue
 results.append(dict(new_home=q.tolist(),actual=actual.tolist(),position_shift_m=dp.tolist(),position_shift_norm_m=float(np.linalg.norm(dp)),rotation_error_deg=float(angle),min_screen_clearance_m=clearance(r,q)))
 if len(results)>=6:break
if not results:raise RuntimeError('no screened 30deg nearby candidate')
results.sort(key=lambda x:x['position_shift_norm_m']);best=results[0]
d=dict(chosen=best,current=current.tolist(),old_home=old.tolist(),original_target=target.tolist(),object_yaw_deg=30,solver='bounded URDF FK least squares; position adjusted, orientation preserved',results=results)
(out/'candidate.json').write_text(json.dumps(d,indent=2));print(json.dumps(best,indent=2))
