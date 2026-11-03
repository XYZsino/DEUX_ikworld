"""Named, measured arm posture for a fixed barcode presentation; no camera mapping."""
import json,hashlib
from pathlib import Path
import numpy as np
from deux_task_core.model import RobotModel
from deux_task_core.planning import hermite
from scipy.spatial.transform import Rotation


def load_pose(filename):
    if not filename:return None
    data=json.loads(Path(filename).read_text())
    if data.get('schema')!='deux.dataset_reading_pose.v1':raise ValueError('unknown dataset reading pose schema')
    if data.get('applied_arms')!='left_only':raise ValueError('dataset posture must apply left arm only')
    for side in ['left']:
        joints=np.asarray(data[side+'_joints_rad'],float)
        if joints.shape!=(7,) or not np.isfinite(joints).all():raise ValueError('seven finite arm joint angles required')
    tail=data.get('alignment_tail')
    if tail:
        joints=np.asarray(tail['target_left_joints_rad'],float)
        if joints.shape!=(7,) or not np.isfinite(joints).all():raise ValueError('invalid alignment tail joints')
        if not np.allclose(tail['start_left_joints_rad'],data['left_joints_rad'],atol=1e-10,rtol=0.):raise ValueError('alignment tail start mismatch')
        if tail['axis']!=[0.,0.,1.] or not np.isfinite(tail['angle_deg']) or abs(tail['angle_deg'])>10.:raise ValueError('invalid base yaw tail')
    return data


def validate_endpoint(pose,positions):
    end=np.asarray(positions,float)[-1]
    if end.shape!=(28,):raise ValueError('28-joint reading path required')
    tail=pose.get('alignment_tail')
    target=tail['target_left_joints_rad'] if tail else pose['left_joints_rad']
    if not np.allclose(end[:7],target,atol=1e-7,rtol=0.):raise ValueError('dataset reading endpoint mismatch: left')
    if tail and not np.any(np.max(abs(np.asarray(positions)[:,:7]-pose['left_joints_rad']),axis=1)<1e-7):raise ValueError('alignment tail must follow original scan pose')
    path=np.asarray(positions,float)
    if not np.allclose(path[:,14:21],path[0,14:21],atol=1e-10,rtol=0.):raise ValueError('existing right arm must remain stationary during reading alignment')


def build_reading_plan(config,pose,actual):
    robot=RobotModel(config['robot']);start=np.asarray(actual,float).copy()
    if start.shape!=(28,) or not np.isfinite(start).all():raise ValueError('finite measured start required')
    if config.get('hand_command_only',False):
        start[7:14]=config['hand_closed'][:7];start[21:28]=config['hand_closed'][7:]
    goal=start.copy();goal[:7]=pose['left_joints_rad']
    for q in [start,goal]:
        if np.any(q<robot.limits[:,0]-robot.joint_limit_tolerance) or np.any(q>robot.limits[:,1]+robot.joint_limit_tolerance):raise ValueError('reading posture joint limit')
    scale=float(config.get('scan_motion_scale',.25));t,path=hermite([start,goal],robot.velocity*scale,robot.acceleration*scale**2,config['sample_period'])
    tail=pose.get('alignment_tail');tail_begin=None
    if tail:
        if tail['frame']!=config['base_frame'] or tail['fixed_joints']!=robot.fixed or tail['urdf_sha256']!=hashlib.sha256(Path(config['robot']['urdf_path']).read_bytes()).hexdigest():raise ValueError('alignment tail model/frame changed; regenerate')
        if not np.allclose(tail['start_left_joints_rad'],goal[:7],atol=1e-10,rtol=0.):raise ValueError('alignment tail start mismatch')
        final=goal.copy();final[:7]=tail['target_left_joints_rad']
        if np.any(final<robot.limits[:,0]-robot.joint_limit_tolerance) or np.any(final>robot.limits[:,1]+robot.joint_limit_tolerance):raise ValueError('alignment tail joint limit')
        initial=robot.ee_fk('left',goal);expected=initial.copy()
        expected[:3,:3]=Rotation.from_euler('z',tail['angle_deg'],degrees=True).as_matrix()@initial[:3,:3]
        actual_end=robot.ee_fk('left',final)
        if np.linalg.norm(actual_end[:3,3]-expected[:3,3])>min(.001,float(tail['position_tolerance_m'])) or np.linalg.norm(Rotation.from_matrix(actual_end[:3,:3]@expected[:3,:3].T).as_rotvec())>min(.002,float(tail['rotation_tolerance_rad'])):raise ValueError('alignment tail IK target mismatch')
        tt,qq=hermite([goal,final],robot.velocity*scale,robot.acceleration*scale**2,config['sample_period'],minimum_segment_duration=float(tail['minimum_duration_s']))
        for row in qq:
            pose_now=robot.ee_fk('left',row)
            if np.linalg.norm(pose_now[:3,3]-initial[:3,3])>min(.001,float(tail['position_tolerance_m'])):raise ValueError('alignment tail EE position drift')
            rot=Rotation.from_matrix(pose_now[:3,:3]@initial[:3,:3].T).as_rotvec()
            if np.linalg.norm(rot[:2])>min(.002,float(tail['rotation_tolerance_rad'])):raise ValueError('alignment tail rotation leaves base Z axis')
        # Hand closes during the original alignment; rotate only after closure.
        tail_begin=max(float(t[-1]),float(config.get('barcode_align_left_close_duration_s',.5))+.1)
        if tail_begin>t[-1]+1e-9:
            hold=np.linspace(t[-1],tail_begin,int(np.ceil((tail_begin-t[-1])/config['sample_period']))+1)[1:]
            t=np.r_[t,hold];path=np.vstack([path,np.tile(goal,(len(hold),1))])
        t=np.r_[t,tail_begin+tt[1:]];path=np.vstack([path,qq[1:]])
    validate_endpoint(pose,path)
    return dict(times=t,path=path,minimum_clearance=0.,ik_calls=0,alignment_tail_start_s=tail_begin,

        dataset_reading_reference=dict(episode=pose['episode'],frame=pose['frame'],timestamp_s=pose['timestamp_s']))
