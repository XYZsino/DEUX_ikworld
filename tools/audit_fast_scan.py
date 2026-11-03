"""Offline whole-library fast scan audit. No ROS imports or hardware commands."""
import argparse,json,time,os
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import numpy as np,yaml
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_fast_scan import ROOT,REPORT,PROFILE,fingerprint,compile_tail
LIB=CONFIG=LEFT=None

def check(key):
    try:
        _,_,info=compile_tail(LIB,key,CONFIG,left_target=LEFT)
        return key,{k:info[k] for k in ['minimum_conveyor_clearance_m','tail_duration_s','waypoint_at_s','duration_s','original_duration_s']},None
    except ValueError as error:return key,None,str(error)

def main():
    global LIB,CONFIG,LEFT
    parser=argparse.ArgumentParser();parser.add_argument('--workers',type=int,default=12);args=parser.parse_args()
    path=ROOT/'src/deux_task_bringup/config/installation.yaml';CONFIG=yaml.safe_load(path.read_text())
    CONFIG['robot']['urdf_path']=str((path.parent/CONFIG['robot']['urdf_path']).resolve())
    LIB=ClassifiedGridLibrary(CONFIG,directory=ROOT/'artifacts/right_grasp_grid_classified',recommended=False,yaw_secondary=True,load_rotation_paths=False)
    LIB.ready[:7]=CONFIG['home_joints'][:7]
    LEFT=json.loads((ROOT/'artifacts/dataset-barcode-pose/reading-pose.json').read_text())['left_joints_rad']
    accepted={};rejected={};began=time.monotonic()
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('fork')) as pool:
        for i,(key,row,error) in enumerate(pool.map(check,LIB.entries,chunksize=8),1):
            if error:rejected[key]=error
            else:accepted[key]=row
            if i%100==0:print(f'{i}/{len(LIB.entries)} accepted={len(accepted)} rejected={len(rejected)} elapsed={time.monotonic()-began:.1f}s',flush=True)
    record=dict(profile=PROFILE,fingerprint=fingerprint(LIB,CONFIG,LEFT),accepted=accepted,rejected=rejected,
        total=len(LIB.entries),accepted_count=len(accepted),rejected_count=len(rejected),
        target_tail_s=1.8,right_prefix_unchanged_through_via=True,left_starts_at_via=True,hardware_verified=False,
        checks='Both arms joint bounds, exact transmitted cubic velocity/acceleration extrema; continuous conveyor-only clearance bound. Original right prefix through via retained.',
        elapsed_s=time.monotonic()-began)
    REPORT.write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps({k:v for k,v in record.items() if k not in ('accepted','rejected')},indent=2))
if __name__=='__main__':main()
