"""Explicit stationary MuJoCo test installation, never hardware calibration."""
import sys
from pathlib import Path
import numpy as np


def load_sim(repository, output, seed=4):
    repository=Path(repository).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    sys.path[:0]=[str(repository/'sim'),str(repository/'vendor')]
    import deux_mjmodel as robot
    original=robot.write_fixed_urdf
    robot.write_fixed_urdf=lambda:original(str(output/'robot.urdf'))
    from reader_pick import create_sim
    try: sim=create_sim(seed,box_sources=True,moving_boxes=False)
    finally: robot.write_fixed_urdf=original
    # loop_pick_scene raises the fixed lift AFTER URDF import. Mirror that
    # explicit scene modification in the planning URDF, then verify both FK paths.
    import xml.etree.ElementTree as ET
    from loop_pick_scene import ROBOT_LIFT_RAISE
    urdf=output/'robot.urdf';tree=ET.parse(urdf)
    origin=tree.getroot().find("joint[@name='lift_joint']/origin")
    xyz=np.array(origin.get('xyz').split(),float);xyz[2]+=ROBOT_LIFT_RAISE
    origin.set('xyz',' '.join(map(str,xyz)));tree.write(urdf);urdf.chmod(0o644)
    sim.conveyor.set_speed(0.)
    # Fixed test installation: translate the right source tray and its contents
    # together before the run. No object teleporting or welds during execution.
    import mujoco
    sim.model.body_pos[sim.model.body('source_box_2').id,1]+=.10
    for name in ('bread_3','bread_4'):
        j=sim.model.joint(name+'_free');sim.data.qpos[int(j.qposadr[0])+1]+=.10
    mujoco.mj_forward(sim.model,sim.data)
    sim.command(sim.home)
    for _ in range(15): sim.advance()
    mujoco.mj_forward(sim.model,sim.data)
    return sim


def configure(sim, output):
    import yaml
    from transfer_sim_adapter import SimObservationSource, transform
    from transfer_contact_planner import ObservedContactPlanner
    from deux_task_core.model import RobotModel, JOINT_NAMES, HAND_NAMES
    source=SimObservationSource(sim,width=160,height=90,min_visible_pixels=4)
    try: frame=source.capture(cameras=('head','right_wrist'))
    finally: source.close()
    obj=next(d for d in frame.detections if d.track_id=='bread_3')
    grasp=ObservedContactPlanner(sim.model,adaptive_pitch=True).plan(frame,obj.pose)
    output=Path(output)
    robot=dict(urdf_path=str(output/'robot.urdf'),base_frame='base_link',
        joint_limit_tolerance=1e-4,ee_links={'left':'left_grasp','right':'right_grasp'},acceleration_limits=[3.]*28,
        collision_spheres=[dict(link=f'{side}_grasp',center=[0.,0.,0.],radius=.035) for side in ('left','right')],
        attachment_touch_links=['right_grasp'])
    model=RobotModel(robot)
    fk=model.fk(frame.joints)
    for side,pose in [('left',frame.ee_left),('right',frame.ee_right)]:
        if not np.allclose(fk[side+'_grasp'],pose,atol=1e-6):raise ValueError(f'URDF/MuJoCo FK mismatch {side}: urdf={fk[side+chr(95)+"grasp"].tolist()}, sim={pose.tolist()}')
    base=np.linalg.inv(transform(sim.data,'base_link'))
    sid=sim.model.site('left_fixed_reader_scan_origin').id
    scan=np.eye(4);scan[:3,:3]=sim.data.site_xmat[sid].reshape(3,3);scan[:3,3]=sim.data.site_xpos[sid]
    # ROS reader frame is +Z forward, legacy sim scanner is -Z forward.
    reader=np.linalg.inv(frame.ee_left)@base@scan@np.diag([1.,-1.,-1.,1.])
    dest=obj.pose.copy();dest[:3,3]=(base@transform(sim.data,'right_box'))[:3,3]+[0.,0.,.17]
    opened=np.r_[frame.joints[7:14],grasp['approach_hand']]
    # Static simulation-only preload calibration, derived from the known
    # contact geometry and actuator stiffness; never read live contact force.
    from dataclasses import replace
    from transfer_motion import ObservedRobotMotion
    from transfer_grip import ObservedGripFeedforward
    predicted=model.ik('right',grasp['ee_pose'],frame.joints)
    predicted[21:28]=grasp['closed_hand']
    nominal=replace(frame,joints=predicted,ee_right=grasp['ee_pose'])
    loaded=ObservedGripFeedforward(ObservedRobotMotion(sim.model,.034)).target(nominal,obj.pose,grasp)
    closed=np.r_[frame.joints[7:14],loaded]
    grasp['record']['ros_static_preload_offsets_rad']=(loaded-grasp['closed_hand']).tolist()
    c=dict(schema='deux.scene.v1',synthetic=True,simulation_wall_timeout_scale=8.,base_frame='base_link',model_id='sim-mujoco-v6-centred',robot=robot,
        source_box_height=.07,grasp_object_from_right_ee=grasp['object_from_ee'].tolist(),
        scan_right_pose=frame.ee_right.tolist(),scan_left_pose=frame.ee_left.tolist(),
        retreat_left_pose=frame.ee_left.tolist(),destination_pose=dest.tolist(),
        left_ee_from_reader=reader.tolist(),read_distance=.12,
        hand_open=opened.tolist(),hand_closed=closed.tolist(),home_joints=frame.joints.tolist(),
        workspace=dict(min=[-.6,-1.,.1],max=[1.2,1.,1.6]),
        destination_bounds=dict(min=[-.127,-.695,.505],max=[.143,-.425,.715]),
        obstacles=[],safe_joint_waypoint=None,approach_height=.10,lift_height=.13,attached_radius=.10,
        max_target_drift=.025,max_target_angle_rad=.25,max_observation_age=.5,max_joint_age=.3,
        max_object_age=.6,max_plan_age=3.,sample_period=.02,collision_margin=.005,
        start_tolerance=.15,tracking_tolerance=.6,settle_timeout=2.,scan_timeout=2.,verification_timeout=2.)
    (output/'installation.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
    mapping=dict(calibrated=True,position_mode_confirmed=True)
    for side in ('left','right'):
        names=[n for n in HAND_NAMES if n.startswith(side)]
        mapping[side]=dict(feedback_topic=f'/sim/hand/{side}/joint_states',feedback_type='sensor_msgs/msg/JointState',
            mode=7,action_state=0,action_state_save=0,joints={n:dict(id=i,feedback_name=n,sign=1,offset=0.,
              min=float(model.limits[JOINT_NAMES.index(n),0])-1e-4,max=float(model.limits[JOINT_NAMES.index(n),1])+1e-4,
              velocity=float(model.velocity[JOINT_NAMES.index(n)]),goal_current=0.) for i,n in enumerate(names)})
    (output/'hand_mapping.yaml').write_text(yaml.safe_dump(mapping))
    return c,frame,grasp
