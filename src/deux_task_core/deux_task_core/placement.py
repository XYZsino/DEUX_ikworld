"""Calibrated single-bin placement; never substitutes stale ICP for held pose."""
import numpy as np
from .model import pose, vector

PLACEMENT_STAGES = ('ClearReader', 'PlaceApproach', 'Place', 'Release', 'PlaceRetreat')
LOADED_STAGES = ('Lift', 'Scan', 'ScanPose', 'AlignBarcode', 'ClearReader', 'PlaceApproach', 'Place', 'Recover', 'ReadyPose')


def enabled(config):
    return config.get('perception_policy') == 'head_rgbd' and bool(config.get('placement', {}).get('enabled', False))


def validate(config):
    p = config['placement']
    if not isinstance(p.get('enabled'), bool):
        raise ValueError('placement.enabled must be boolean')
    if not p['enabled']:
        return
    provisional = (p.get('source') == 'sim_temporary_operator_confirmed'
                   and p.get('allow_uncalibrated_sim') is True)
    demonstrated = (p.get('source') == 'demonstration_temporary_operator_confirmed'
                    and p.get('allow_uncalibrated_demonstration') is True
                    and p.get('fixed_joint_motion') is True)
    provisional = provisional or demonstrated
    if not config.get('synthetic', False) and p.get('calibrated') is not True and not provisional:
        raise ValueError('real placement requires calibrated EE/bin configuration or explicitly confirmed temporary placement')
    if 'descent_ik_rotation_tolerance_rad' in p:
        value=p['descent_ik_rotation_tolerance_rad']
        if not np.isfinite(value) or not .02 <= value <= np.pi/6:
            raise ValueError('invalid placement descent IK rotation tolerance')
    if type(p.get('skip_fixed_motion_collision_checks', False)) is not bool:
        raise ValueError('placement.skip_fixed_motion_collision_checks must be boolean')
    if type(p.get('fixed_joint_motion', False)) is not bool:
        raise ValueError('placement.fixed_joint_motion must be boolean')
    if p.get('fixed_joint_motion', False):
        if p.get('release_at_approach', False) or p.get('finish_after_release', False):
            raise ValueError('fixed placement requires complete release and retreat sequence')
        blend=p.get('fixed_joint_corner_blend',0.)
        if not np.isfinite(blend) or not 0 <= blend <= 1:
            raise ValueError('invalid placement corner blend')
        scale=p.get('joint_motion_scale', 1.)
        if not np.isfinite(scale) or not 0 < scale <= 1:
            raise ValueError('invalid placement joint motion scale')
        for stage in ('PlaceApproach', 'Place', 'PlaceRetreat', 'Retreat'):
            points=p.get('fixed_joint_waypoints', {}).get(stage, [])
            if not isinstance(points, list) or not 1 <= len(points) <= 100:
                raise ValueError('missing fixed placement waypoints: '+stage)
            for point in points:
                vector(point, 7)
    if type(p.get('continuous_motion',False)) is not bool:
        raise ValueError('continuous_motion must be boolean')
    if p.get('continuous_motion',False) and not p.get('fixed_joint_motion',False):
        raise ValueError('continuous_motion requires fixed joint motion')
    vector(p['clear_left_joints'], 7)
    hover = pose(p['hover_right_pose'])
    if p.get('release_at_approach', False):
        release = hover
        vector(p['hover_right_joints'], 7)
    else:
        release = pose(p['release_right_pose'])
        # Recorded joint routes can descend diagonally and rotate the wrist.
        # Their measured-start guards use FK of the stored joint endpoints.
        if not demonstrated:
            if not np.allclose(hover[:3, :3], release[:3, :3], atol=1e-5):
                raise ValueError('placement hover and release must share orientation')
            if 'descent_axis_base' in p:
                axis=vector(p['descent_axis_base'],3);delta=hover[:3,3]-release[:3,3]
                distance=float(np.dot(delta,axis))
                if abs(np.linalg.norm(axis)-1)>1e-6 or axis[2]<=0 or not .05<=distance<=.35 or np.linalg.norm(delta-distance*axis)>.002:
                    raise ValueError('invalid camera-relative placement descent')
            elif np.linalg.norm(hover[:2, 3]-release[:2, 3]) > .002 or not .05 <= hover[2, 3]-release[2, 3] <= .35:
                raise ValueError('placement requires vertical hover/release clearance of 5..35cm')
    bounds = config['destination_bounds']
    if np.any(release[:2, 3] < bounds['min'][:2]) or np.any(release[:2, 3] > bounds['max'][:2]):
        raise ValueError('release EE must be over destination XY bounds')
    for key, lo, hi in [('position_tolerance_m', .005, .04), ('rotation_tolerance_rad', .02, np.pi/6),
                        ('left_tolerance_rad', .01, .12), ('hand_tolerance_rad', .02, .2)]:
        x=p[key]
        if not np.isfinite(x) or not lo <= x <= hi:
            raise ValueError('invalid placement '+key)


def check_start(robot, config, stage, joints):
    """Both planner and executor enforce ordered geometry, including direct action clients."""
    if not enabled(config) or (stage not in PLACEMENT_STAGES and not (stage == 'Retreat' and config['placement'].get('fixed_joint_motion', False))):
        return
    from scipy.spatial.transform import Rotation
    p=config['placement'];q=np.asarray(joints)
    if p.get('continuous_motion',False) and stage in ('Place','PlaceRetreat'):
        raise ValueError('separate placement segment disabled in continuous mode')
    if stage != 'ClearReader' and np.max(np.abs(q[:7]-p['clear_left_joints'])) > p['left_tolerance_rad']:
        raise ValueError('placement requires cleared left reader')
    if p.get('release_at_approach', False):
        if stage in ('Place', 'PlaceRetreat'):
            raise ValueError('lower placement waypoints disabled in release-at-approach mode')
        expected = None
        if stage == 'Release':
            reference=q.copy();reference[14:21]=vector(p['hover_right_joints'],7)
            expected=robot.fk(reference)[robot.ee['right']]
    else:
        expected = {'Place':p['hover_right_pose'], 'Release':p['release_right_pose'],
                    'PlaceRetreat':p['release_right_pose']}.get(stage)
    if stage == 'Place' and 'hover_right_joints' in p:
        reference=q.copy();reference[14:21]=vector(p['hover_right_joints'],7)
        expected=robot.fk(reference)[robot.ee['right']]
    if p.get('fixed_joint_motion', False):
        previous={'Place':'PlaceApproach', 'Release':'Place', 'PlaceRetreat':'Place', 'Retreat':'PlaceRetreat'}.get(stage)
        if p.get('continuous_motion',False) and stage=='Retreat':
            previous='Place'
        expected=None
        if previous is not None:
            reference=q.copy()
            reference[14:21]=vector(p['fixed_joint_waypoints'][previous][-1],7)
            expected=robot.fk(reference)[robot.ee['right']]
    if expected is not None:
        actual=robot.fk(q)[robot.ee['right']];expected=pose(expected)
        if np.linalg.norm(actual[:3,3]-expected[:3,3]) > p['position_tolerance_m']:
            raise ValueError('placement start is outside the required waypoint')
        if Rotation.from_matrix(actual[:3,:3]@expected[:3,:3].T).magnitude() > p['rotation_tolerance_rad']:
            raise ValueError('placement start orientation mismatch')
    if not config.get('hand_command_only',False) and stage == 'PlaceRetreat' and np.max(np.abs(q[21:28]-np.asarray(config['hand_open'])[7:14])) > p['hand_tolerance_rad']:
        raise ValueError('placement retreat requires measured open right hand')


def joint_points(config,stage):
    p=config['placement'];points=p['fixed_joint_waypoints']
    if p.get('continuous_motion',False):
        if stage=='PlaceApproach':return points['PlaceApproach']+points['Place']
        if stage=='Retreat':return points['PlaceRetreat']+points['Retreat']
    return points[stage]


def stages(config):
    if config['placement'].get('continuous_motion',False):
        return ('ClearReader','PlaceApproach')
    if config['placement'].get('release_at_approach', False):
        return ('ClearReader', 'PlaceApproach', 'Release')
    if config['placement'].get('finish_after_release',False):
        return ('ClearReader', 'PlaceApproach', 'Place', 'Release')
    return ('ClearReader', 'PlaceApproach', 'Place', 'Release', 'PlaceRetreat', 'Retreat')


def skip_collision(config, stage):
    """Explicit operator opt-out for the fixed bin motion only."""
    return (enabled(config)
            and config['placement'].get('fixed_joint_motion', False)
            and config['placement'].get('skip_fixed_motion_collision_checks', False)
            and stage in ('PlaceApproach', 'Place', 'Release', 'PlaceRetreat', 'Retreat'))


def trajectory_clearance(robot, config, stage, command, attached):
    if skip_collision(config, stage):
        q=vector(command,28)
        if (np.any(q < robot.limits[:,0]-robot.joint_limit_tolerance)
                or np.any(q > robot.limits[:,1]+robot.joint_limit_tolerance)):
            raise ValueError('joint limit')
        # Existing sentinel means no modeled clearance restriction, not measured space.
        return 1e6
    return robot.clearance(command,config['obstacles'],config['collision_margin'],attached)
