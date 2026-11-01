"""Apply a Cartesian scan offset with an exact J6 increment; no robot commands."""
import json,sys,argparse
from pathlib import Path
import numpy as np,yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src/deux_task_core'))
from deux_task_core.model import RobotModel
parser=argparse.ArgumentParser();parser.add_argument('--from-pose');parser.add_argument('--dy',type=float,default=.05);parser.add_argument('--dz',type=float,default=.10);parser.add_argument('--j6-delta-deg',type=float,default=30.);args=parser.parse_args()
out=ROOT/'artifacts/dataset-barcode-pose';file=out/'reading-pose.json';original=Path(args.from_pose) if args.from_pose else out/'reading-pose.before-left-offset.json'
delta=np.array([0.,args.dy,args.dz]);joint_delta=np.deg2rad(args.j6_delta_deg)
if not original.exists():original.write_bytes(file.read_bytes())
d=json.loads(original.read_text());p=ROOT/'src/deux_task_bringup/config/installation.yaml';c=yaml.safe_load(p.read_text());c['robot']['urdf_path']=str((p.parent/c['robot']['urdf_path']).resolve());robot=RobotModel(c['robot'])
old=np.array(c['home_joints']);old[:7]=d['left_joints_rad'];old[14:21]=c['scan_right_joints'];before=robot.ee_fk('left',old);turned=old.copy();turned[5]+=joint_delta
# URDF J6 axis (-X local): +30deg turns the reader yaw toward robot right.
target=robot.ee_fk('left',turned);target[:3,3]=before[:3,3]+delta;indices=np.array([0,1,2,3,4,6]);rng=np.random.default_rng(2026)
def unpack(values):q=turned.copy();q[indices]=values;return q
def residual(values):
 actual=robot.ee_fk('left',unpack(values));return np.r_[20*(actual[:3,3]-target[:3,3]),3*Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).as_rotvec()]
candidates=[]
seeds=[turned[indices]]+[np.clip(turned[indices]+rng.normal(0,.45,6),robot.limits[indices,0]+1e-5,robot.limits[indices,1]-1e-5) for _ in range(14)]
for seed in seeds:
 result=least_squares(residual,seed,bounds=(robot.limits[indices,0]+1e-6,robot.limits[indices,1]-1e-6),max_nfev=260,ftol=1e-11,xtol=1e-11,gtol=1e-11)
 q=unpack(result.x);actual=robot.ee_fk('left',q);pe=np.linalg.norm(actual[:3,3]-target[:3,3]);ae=Rotation.from_matrix(actual[:3,:3]@target[:3,:3].T).magnitude()
 if pe<.001 and ae<np.deg2rad(.5):candidates.append((float(np.linalg.norm(q[:7]-old[:7])),q,pe,ae))
 if candidates and candidates[-1][0]<1.5:break
if not candidates:raise ValueError('No pose satisfies requested translation, requested exact J6 increment and orientation')
_,q,pe,ae=min(candidates,key=lambda x:x[0]);actual=robot.ee_fk('left',q);reader_offset=np.array(c['left_ee_from_reader']);old_reader=before@reader_offset;new_reader=actual@reader_offset
adjustment=dict(frame='base_link',translation_m=delta.tolist(),joint='arm_l_joint6',joint_delta_rad=float(joint_delta),joint_delta_deg=args.j6_delta_deg,right_rotation_sign_reason='URDF J6 local axis is -X; positive J6 decreases reader base yaw in this pose',before_left_joints_rad=old[:7].tolist(),before_left_xyz_m=before[:3,3].tolist(),after_left_xyz_m=actual[:3,3].tolist(),position_error_m=float(pe),orientation_error_deg=float(np.degrees(ae)),reader_yaw_before_deg=float(np.degrees(np.arctan2(old_reader[1,2],old_reader[0,2]))),reader_yaw_after_deg=float(np.degrees(np.arctan2(new_reader[1,2],new_reader[0,2]))),actual_hardware_executed=False)
d['left_joints_rad']=q[:7].tolist();d['ee_xyz_m'][0]=actual[:3,3].tolist();d['adjustment']=adjustment
(out/'reading-pose-offset.json').write_text(json.dumps(d,indent=2));(out/'left-scan-offset-result.json').write_text(json.dumps(adjustment,indent=2));print(json.dumps(dict(**adjustment,left_joints_deg=np.degrees(q[:7]).tolist()),indent=2))
