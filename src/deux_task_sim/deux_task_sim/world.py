"""Explicit stationary MuJoCo test installation, never hardware calibration."""
import sys
from pathlib import Path
import numpy as np


def load_sim(repository, output, seed=4, conveyor_speed=0.):
    repository=Path(repository).resolve();output=Path(output).resolve();output.mkdir(parents=True,exist_ok=True)
    sys.path[:0]=[str(repository/'sim'),str(repository/'vendor')]
    import deux_mjmodel as robot
    original=robot.write_fixed_urdf
    original_extras=robot.add_robot_extras
    from ament_index_python.packages import get_package_share_directory
    import yaml, json
    import xml.etree.ElementTree as ET
    from deux_task_core.camera_mount import append_zed2i
    assets=Path(get_package_share_directory('deux_task_bringup'))
    mount=yaml.safe_load((assets/'config/zed2i_mount.yaml').read_text())
    def mounted_urdf():
        path=original(str(output/'robot.urdf'))
        tree=ET.parse(path)
        audit=append_zed2i(tree.getroot(),mount,assets/'meshes/zed2i.stl')
        tree.write(path)
        (output/'camera-mount.json').write_text(json.dumps(dict(config=mount,audit=audit),indent=2))
        return path
    def mounted_extras(spec):
        original_extras(spec)
        fovy=np.rad2deg(2*np.arctan(np.tan(np.deg2rad(mount['sim_horizontal_fov_deg'])/2)
                                   *mount['sim_height']/mount['sim_width']))
        for side in ('left','right'):
            spec.body(f"{mount['name']}_{side}_camera_frame_optical").add_camera(
                name=f'zed2i_{side}',pos=[0,0,0],quat=[0,1,0,0],fovy=fovy)
    robot.write_fixed_urdf=mounted_urdf
    robot.add_robot_extras=mounted_extras
    from reader_pick import create_sim
    try: sim=create_sim(seed,box_sources=True,moving_boxes=bool(conveyor_speed))
    finally:
        robot.write_fixed_urdf=original
        robot.add_robot_extras=original_extras
    sim.zed_camera_config=mount
    sim.configured_conveyor_speed=float(conveyor_speed)
    # Shift only the robot; conveyor and boxes retain their world poses.
    sim.model.body_pos[sim.model.body("base_link").id, 0] -= .20

    # loop_pick_scene raises the fixed lift AFTER URDF import. Mirror that
    # explicit scene modification in the planning URDF, then verify both FK paths.
    import xml.etree.ElementTree as ET
    from loop_pick_scene import ROBOT_LIFT_RAISE
    urdf=output/'robot.urdf';tree=ET.parse(urdf)
    origin=tree.getroot().find("joint[@name='lift_joint']/origin")
    xyz=np.array(origin.get('xyz').split(),float);xyz[2]+=ROBOT_LIFT_RAISE
    origin.set('xyz',' '.join(map(str,xyz)));tree.write(urdf);urdf.chmod(0o644)
    sim.conveyor.set_speed(conveyor_speed)
    # Fixed test installation: translate the right source tray and its contents
    # together before the run. No object teleporting or welds during execution.
    import mujoco
    if conveyor_speed:
        j=sim.model.joint('source_box_2_free');sim.data.qpos[int(j.qposadr[0])+1]+=.10
    else:sim.model.body_pos[sim.model.body('source_box_2').id,1]+=.10
    for name in ('bread_3','bread_4'):
        j=sim.model.joint(name+'_free');sim.data.qpos[int(j.qposadr[0])+1]+=.10
    if conveyor_speed:
        # Initial upstream staging only: the belt runs throughout preparation
        # and grasp. No target/body transforms are supplied to the controller.
        for name in ('source_box_1','bread_1','bread_2'):
            j=sim.model.joint(name+'_free');sim.data.qpos[int(j.qposadr[0])+1]+=.25
    mujoco.mj_forward(sim.model,sim.data)
    sim.command(sim.home)
    for _ in range(15): sim.advance()
    mujoco.mj_forward(sim.model,sim.data)
    return sim


def configure(sim, output):
    import yaml
    from .ready_pose import HOME_JOINTS, ATTENTION_CLEARANCE_JOINTS
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
        joint_limit_tolerance=1e-3,ee_links={'left':'left_grasp','right':'right_grasp'},acceleration_limits=[3.]*28,
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
    box_position=(base@transform(sim.data,'right_box'))[:3,3]
    dest=obj.pose.copy();dest[:3,3]=box_position+[0.,0.,.17]
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
    ready_fk = model.fk(np.asarray(HOME_JOINTS))
    # Overlap shoulder, elbow and hand motion through one clearance waypoint.
    # Keep the final attention-reference EE XY and existing dynamic limits.
    coordinated = np.asarray(ATTENTION_CLEARANCE_JOINTS) * .55
    coordinated[[0, 14]] = [.85, -.85]
    coordinated[[3, 17]] = [1.05, -1.05]
    c=dict(prepare_joint_waypoints=[coordinated.tolist(), ATTENTION_CLEARANCE_JOINTS],schema='deux.scene.v1',synthetic=True,simulation_wall_timeout_scale=8.,base_frame='base_link',model_id='sim-mujoco-v6-centred',robot=robot,
        source_box_height=.07,grasp_object_from_right_ee=grasp['object_from_ee'].tolist(),
        scan_right_pose=ready_fk['right_grasp'].tolist(),scan_left_pose=ready_fk['left_grasp'].tolist(),
        retreat_left_pose=ready_fk['left_grasp'].tolist(),destination_pose=dest.tolist(),
        left_ee_from_reader=reader.tolist(),read_distance=.12,
        hand_open=opened.tolist(),hand_closed=closed.tolist(),home_joints=HOME_JOINTS,prepare_before_first_task=True,
        workspace=dict(min=[-.6,-1.,.1],max=[1.2,1.,1.6]),
        destination_bounds=dict(min=(box_position+[-.135,-.135,.02]).tolist(),
                                max=(box_position+[.135,.135,.23]).tolist()),
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
              min=float(model.limits[JOINT_NAMES.index(n),0])-1e-3,max=float(model.limits[JOINT_NAMES.index(n),1])+1e-3,
              velocity=float(model.velocity[JOINT_NAMES.index(n)]),goal_current=0.) for i,n in enumerate(names)})
    (output/'hand_mapping.yaml').write_text(yaml.safe_dump(mapping))
    return c,frame,grasp


def initialize_urdf_zero(sim):
    """Set initial robot state once, before ROS starts; never reset during tasks."""
    import mujoco
    from deux_task_core.model import JOINT_NAMES
    for name in JOINT_NAMES:
        joint = sim.model.joint(name)
        sim.data.qpos[int(joint.qposadr[0])] = 0.
    sim.data.qvel[:] = 0.
    sim.data.qacc_warmstart[:] = 0.
    sim.data.time = 0.
    sim.finger_targets = {side: np.zeros(7) for side in ('left', 'right')}
    sim.command(np.zeros(16))
    for side in ('left', 'right'):
        sim.data.ctrl[sim._hand_u[side]] = 0.
    mujoco.mj_forward(sim.model, sim.data)


def ready_pose_comparison(sim, config, joints=None):
    """Measured world EE/elbow pose against pre-retreat URDF attention (joint-zero)."""
    from scipy.spatial.transform import Rotation
    from deux_task_core.model import RobotModel
    from transfer_sim_adapter import transform
    from .ready_pose import ATTENTION_JOINTS
    model = RobotModel(config['robot'])
    reference = model.fk(np.asarray(ATTENTION_JOINTS))
    world_from_base = transform(sim.data, 'base_link').copy()
    measured_fk = None if joints is None else model.fk(np.asarray(joints))
    original_base = world_from_base.copy()
    original_base[0, 3] += .20
    result = {'reference': 'URDF attention posture (all joints zero) before the 0.20 m robot base retreat; world coordinates',
              'elbow_frame': 'arm_joint4 pivot (left_arm_4/right_arm_4)', 'sides': {}}
    for side in ('left', 'right'):
        expected = original_base @ reference[side + '_grasp']
        site = sim._grasp_site[side]
        actual = np.eye(4)
        actual[:3, :3] = sim.data.site_xmat[site].reshape(3, 3)
        actual[:3, 3] = sim.data.site_xpos[site]
        expected_elbow = (original_base @ reference[side + '_arm_4'])[:3, 3]
        actual_elbow = transform(sim.data, side + '_arm_4')[:3, 3]
        if measured_fk is not None:
            actual = world_from_base @ measured_fk[side + '_grasp']
            actual_elbow = (world_from_base @ measured_fk[side + '_arm_4'])[:3, 3]
        delta = actual[:3, 3] - expected[:3, 3]
        result['sides'][side] = dict(
            reference_ee_world_m=expected[:3, 3].tolist(), measured_ee_world_m=actual[:3, 3].tolist(),
            ee_delta_world_m=delta.tolist(), ee_xy_error_m=float(np.linalg.norm(delta[:2])),
            ee_orientation_error_rad=float(Rotation.from_matrix(expected[:3, :3] @ actual[:3, :3].T).magnitude()),
            reference_elbow_world_m=expected_elbow.tolist(), measured_elbow_world_m=actual_elbow.tolist(),
            elbow_delta_world_m=(actual_elbow - expected_elbow).tolist())
    return result
