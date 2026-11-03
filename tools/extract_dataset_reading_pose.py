"""Inspect one downloaded episode using named joint mapping and real URDF FK."""
import sys,json
from pathlib import Path
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=['/tmp/deux-dataset-reader-py310',str(ROOT/'src/deux_task_core')]
import pyarrow.parquet as pq
from deux_task_core.model import RobotModel
out=ROOT/'artifacts/dataset-barcode-pose';src=out/'source';p=ROOT/'src/deux_task_bringup/config/installation.yaml';c=yaml.safe_load(p.read_text());c['robot']['urdf_path']=str((p.parent/c['robot']['urdf_path']).resolve());robot=RobotModel(c['robot']);info=json.loads((src/'meta/info.json').read_text());names=info['features']['observation.state']['names'];data=pq.read_table(src/'data/chunk-000/episode_000000.parquet').to_pydict();state=np.asarray(data['observation.state']);t=np.asarray(data['timestamp']);qs=np.tile(c['home_joints'],(len(t),1))
for side,dest in [('l',slice(0,7)),('r',slice(14,21))]:qs[:,dest]=state[:,[names.index(f'arm_{side}_joint{i}_rad') for i in range(1,8)]]
xyz=np.array([[robot.ee_fk(side,q)[:3,3] for side in ['left','right']] for q in qs]);x=xyz[:,:,0]
score=np.min(x,axis=1);global_min_index=int(np.argmax(score));sumidx=int(np.argmax(x.sum(axis=1)))
# Window verified against this episode's ego/left-camera video, not inferred from X alone.
arm=np.r_[0:7,14:21];speed=np.max(abs(np.gradient(qs[:,arm],t,axis=0)),axis=1)
mask=(t>=7.)&(t<=14.)&(speed<.1);eligible=np.flatnonzero(mask)
if not len(eligible):raise ValueError('no stable reading sample')
idx=int(eligible[np.argmax(x[eligible].sum(axis=1))])
report=dict(source=json.loads((src/'revision.json').read_text()),episode=0,frames=len(t),duration_s=float(t[-1]),selection_rule='video-confirmed 7..14s reading window; max(left_X+right_X) with all arm joint speeds below 0.1rad/s',global_min_X_max_frame=global_min_index,selected_frame=idx,timestamp_s=float(t[idx]),ee_xyz_m=xyz[idx].tolist(),left_joints_rad=qs[idx,:7].tolist(),right_joints_rad=qs[idx,14:21].tolist(),max_sum_frame=sumidx,max_sum_time_s=float(t[sumidx]),max_sum_xyz=xyz[sumidx].tolist(),individual_max_indices=np.argmax(x,axis=0).tolist(),actual_hardware_executed=False)
pose=dict(schema='deux.dataset_reading_pose.v1',applied_arms='left_only',right_pose_policy='hold_measured_existing_scan',source=report['source'],episode=0,frame=idx,timestamp_s=float(t[idx]),left_joints_rad=qs[idx,:7].tolist(),right_joints_rad=qs[idx,14:21].tolist(),ee_xyz_m=xyz[idx].tolist(),add_extra_yaw=False,add_j6_bend=False,barcode_read_success_verified=False)
(out/'reading-pose.json').write_text(json.dumps(pose,indent=2))
(out/'selection.json').write_text(json.dumps(report,indent=2));np.savez(out/'episode-analysis.npz',times=t,positions=qs,ee_xyz=xyz);print(json.dumps(report,indent=2));print('top candidates',[(int(i),float(t[i]),x[i].tolist()) for i in np.argsort(score)[-10:]])
