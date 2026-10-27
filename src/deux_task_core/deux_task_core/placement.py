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
    if not config.get('synthetic', False) and p.get('calibrated') is not True:
        raise ValueError('real placement requires calibrated EE/bin configuration')
    vector(p['clear_left_joints'], 7)
    hover, release = pose(p['hover_right_pose']), pose(p['release_right_pose'])
    if not np.allclose(hover[:3, :3], release[:3, :3], atol=1e-5):
        raise ValueError('placement hover and release must share orientation')
    if np.linalg.norm(hover[:2, 3]-release[:2, 3]) > .002 or not .05 <= hover[2, 3]-release[2, 3] <= .35:
        raise ValueError('placement requires vertical hover/release clearance of 5..35cm')
    bounds = config['destination_bounds']
    if np.any(release[:2, 3] < bounds['min'][:2]) or np.any(release[:2, 3] > bounds['max'][:2]):
        raise ValueError('release EE must be over destination XY bounds')
    for key, lo, hi in [('position_tolerance_m', .005, .04), ('rotation_tolerance_rad', .02, .2),
                        ('left_tolerance_rad', .01, .12), ('hand_tolerance_rad', .02, .2)]:
        x=p[key]
        if not np.isfinite(x) or not lo <= x <= hi:
            raise ValueError('invalid placement '+key)


def check_start(robot, config, stage, joints):
    """Both planner and executor enforce ordered geometry, including direct action clients."""
    if not enabled(config) or stage not in PLACEMENT_STAGES:
        return
    from scipy.spatial.transform import Rotation
    p=config['placement'];q=np.asarray(joints)
    if stage != 'ClearReader' and np.max(np.abs(q[:7]-p['clear_left_joints'])) > p['left_tolerance_rad']:
        raise ValueError('placement requires cleared left reader')
    expected = {'Place':p['hover_right_pose'], 'Release':p['release_right_pose'],
                'PlaceRetreat':p['release_right_pose']}.get(stage)
    if expected is not None:
        actual=robot.fk(q)[robot.ee['right']];expected=pose(expected)
        if np.linalg.norm(actual[:3,3]-expected[:3,3]) > p['position_tolerance_m']:
            raise ValueError('placement start is outside the required waypoint')
        if Rotation.from_matrix(actual[:3,:3]@expected[:3,:3].T).magnitude() > p['rotation_tolerance_rad']:
            raise ValueError('placement start orientation mismatch')
    if stage == 'PlaceRetreat' and np.max(np.abs(q[21:28]-np.asarray(config['hand_open'])[7:14])) > p['hand_tolerance_rad']:
        raise ValueError('placement retreat requires measured open right hand')
