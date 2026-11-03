"""Read actual left joints and screen ALL rotation paths. No command interfaces."""
import json
import argparse
from pathlib import Path
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from grid_grasp_runtime import GridLibrary, ROOT, sha
from upper_collision_batch import checked_clearances


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library-directory',type=Path,default=ROOT/'artifacts/right_grasp_grid_rotations')
    parser.add_argument('--output-directory',type=Path,default=ROOT/'.setup/grid-grasp-rotation-test')
    args=parser.parse_args()
    rclpy.init(); node=Node('rotation_grid_left_readonly_audit'); latest={}
    node.create_subscription(JointState,'/arm/joint_states',
        lambda m:latest.update(msg=m,received=time.monotonic()),qos_profile_sensor_data)
    def read():
        until=time.monotonic()+2.
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.03)
        if 'msg' not in latest or time.monotonic()-latest['received']>.3:raise RuntimeError('fresh joints missing')
        msg=latest['msg']; stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if not 0<=node.get_clock().now().nanoseconds/1e9-stamp<=.3:raise RuntimeError('stale joint timestamp')
        values=dict(zip(msg.name,msg.position))
        return np.array([values[f'arm_l_joint{i}'] for i in range(1,8)]),np.array([values[f'arm_r_joint{i}'] for i in range(1,8)])
    try:
        left,right=read()
        path=ROOT/'src/deux_task_bringup/config/installation.yaml'; config=yaml.safe_load(path.read_text())
        config['robot']['urdf_path']=str((path.parent/config['robot']['urdf_path']).resolve())
        directory=args.library_directory
        library=GridLibrary(config,directory=directory,recommended=False,yaw_secondary=True,load_rotation_paths=False)
        times=np.linspace(0.,5.,501)
        def check(key):
            states=np.tile(library.ready,(len(times),1));states[:,:7]=left
            states[:,14:21]=library.curves[key](times)
            try:return key,float(np.min(checked_clearances(library.robot,states,config['obstacles'],config['collision_margin']))),None
            except ValueError as error:return key,None,str(error)
        validated={};rejected={}
        with ThreadPoolExecutor(max_workers=6) as pool:
            futures=[pool.submit(check,key) for key in library.entries]
            for count,future in enumerate(as_completed(futures),1):
                key,margin,error=future.result()
                if error is None:validated[key]=margin
                else:rejected[key]=error
                if count%100==0:print(json.dumps(dict(audited=count,total=len(futures))),flush=True)
        after,right=read()
        if np.max(abs(after-left))>.03:raise RuntimeError('left arm changed during audit; rerun')
        if not validated:raise RuntimeError('no path passed actual-left screening')
        result=dict(left_arm_rad=left.tolist(),manifest_sha256=sha(directory/'manifest.json'),
                    validated_cells=validated,rejected_cells=rejected,complete_library_audit=True,
                    created_at_s=node.get_clock().now().nanoseconds/1e9,
                    left_change_rad=float(np.max(abs(after-left))),
                    right_ready_error_rad=float(np.max(abs(right-library.ready[14:21]))),
                    geometry='existing model and measured visible surface; actual fixed left snapshot',
                    arm_or_hand_commands_issued=False)
        out=args.output_directory;out.mkdir(parents=True,exist_ok=True)
        temporary=out/'stationary-left.tmp';temporary.write_text(json.dumps(result,indent=2));temporary.replace(out/'stationary-left.json')
        print(json.dumps(dict(passed=len(validated),rejected=len(rejected),right_ready_error_rad=result['right_ready_error_rad'],
                              session=str(out/'stationary-left.json'),no_motion_commands=True)),flush=True)
    finally:node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
