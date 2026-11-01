"""Offline full-library transport audit; does not start ROS or hardware."""
import json,sys,time
from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1];sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'src/deux_task_core')]
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_lift_scan import compile_library
p=ROOT/'src/deux_task_bringup/config/installation.yaml';config=yaml.safe_load(p.read_text());config['robot']['urdf_path']=str((p.parent/config['robot']['urdf_path']).resolve())
lib=ClassifiedGridLibrary(config,directory=ROOT/'artifacts/right_grasp_grid_classified',recommended=False,yaw_secondary=True,load_rotation_paths=False)
# Installation home-left is required by the full BT, independent of the original snapshot.
lib.ready[:7]=config['home_joints'][:7]
left=json.loads((ROOT/'artifacts/dataset-barcode-pose/reading-pose.json').read_text())['left_joints_rad']
start=time.monotonic();accepted,rejected=compile_library(lib,config,lambda *x:print(x,flush=True),left_target=left)
report=dict(parallel_left_scan=True,left_scan_start_s=4.,left_goal_source='dataset episode0 frame243',accepted=len(accepted),rejected=rejected,total=len(lib.entries),actual_hardware_executed=False,
 maximum_duration_s=max(v[2]['duration_s'] for v in accepted.values()),
 minimum_clearance_m=min(v[2]['minimum_conveyor_clearance_m'] for v in accepted.values()),elapsed_s=time.monotonic()-start,
 normal_accepted=sum(lib.entries[k].get('selection_class')!='transverse' for k in accepted),transverse_accepted=sum(lib.entries[k].get('selection_class')=='transverse' for k in accepted))
out=ROOT/'artifacts/right-high-scan-waypoint/library-validation.json';out.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
