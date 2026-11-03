"""Offline installation-left-home profile; measured conveyor only. No ROS imports."""
import argparse,json,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import numpy as np
import yaml
from grid_grasp_runtime import ROOT,GridLibrary,sha
from lookup_conveyor import arm_clearance
from upper_collision_batch import checked_clearances


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=ROOT/'.setup/lookup-bt/home-left-conveyor.json')
    args=parser.parse_args();path=ROOT/'src/deux_task_bringup/config/installation.yaml'
    config=yaml.safe_load(path.read_text());config['robot']['urdf_path']=str((path.parent/config['robot']['urdf_path']).resolve())
    directory=ROOT/'artifacts/right_grasp_grid_classified'
    library=GridLibrary(config,directory=directory,recommended=False,yaw_secondary=True,load_rotation_paths=False)
    left=np.asarray(config['home_joints'][:7]);ready=library.ready.copy();ready[:7]=left
    library.robot.collision_check_enabled=False
    left_margin=arm_clearance(library.robot,ready,'left')
    if left_margin<0:raise ValueError('installation left home intersects conveyor')
    times=np.linspace(0.,5.,501)
    def check(key):
        states=np.tile(ready,(len(times),1));states[:,14:21]=library.curves[key](times)
        try:return key,min(left_margin,float(np.min(checked_clearances(library.robot,states,[],config['collision_margin'])))),None
        except ValueError as error:return key,None,str(error)
    accepted={};rejected={}
    with ThreadPoolExecutor(max_workers=6) as pool:
        futures=[pool.submit(check,key) for key in library.entries]
        for count,future in enumerate(as_completed(futures),1):
            key,margin,error=future.result()
            if error is None:accepted[key]=margin
            else:rejected[key]=error
            if count%250==0:print(json.dumps(dict(checked=count,total=len(futures))),flush=True)
    if not accepted:raise ValueError('no conveyor-clear lookup paths')
    record=dict(left_arm_rad=left.tolist(),left_source='installation.yaml home_joints[:7]',
        manifest_sha256=sha(directory/'manifest.json'),validated_cells=accepted,rejected_cells=rejected,
        complete_library_audit=True,created_wall_s=time.time(),left_conveyor_clearance_m=left_margin,
        collision_policy='measured front conveyor only; inter-arm/self/scene-box checks omitted by explicit user instruction',
        screening_dt_s=.01,arm_or_hand_commands_issued=False)
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(record,indent=2))
    print(json.dumps(dict(profile=str(args.output),passed=len(accepted),rejected=len(rejected),left_conveyor_clearance_m=left_margin)),flush=True)


if __name__=='__main__':main()
