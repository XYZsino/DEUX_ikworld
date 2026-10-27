"""Barcode aiming with the last three left-arm joints fixed at ready values.

The configured reader frame uses +Z forward and an explicit body-up axis.
Only shoulder/elbow joints 1..4 are optimized; wrist angles are never relaxed.
"""
import numpy as np
from scipy.optimize import least_squares
from .model import pose, vector


def validate_settings(settings):
    vector(settings['joint_values'],3)
    if not np.isclose(np.linalg.norm(vector(settings['up_axis'],3)),1.):
        raise ValueError('reader up axis must be unit length')
    near,far=vector(settings['distance_range_m'],2)
    if not 0<near<settings['preferred_min_distance_m']<far:
        raise ValueError('invalid fixed-wrist reader range')
    for key,upper in [('max_tilt_deg',15.),('max_path_tilt_deg',20.),('max_aim_deg',6.),('max_view_deg',60.),('start_tolerance_rad',.1)]:
        if not np.isfinite(settings[key]) or not 0<settings[key]<=upper:
            raise ValueError('invalid fixed-wrist '+key)
    if settings['max_tilt_deg']>settings['max_path_tilt_deg']:
        raise ValueError('reader endpoint tilt exceeds path limit')


def reader_frame(robot,q,config):
    return robot.ee_fk('left',q)@pose(config['left_ee_from_reader'])


def check_posture(robot,q,config,*,command=False,endpoint=False):
    cfg=config['barcode_fixed_wrist']
    tolerance=1e-8 if command else cfg['start_tolerance_rad']
    if np.max(abs(np.asarray(q)[4:7]-cfg['joint_values']))>tolerance:
        raise ValueError('left scanner wrist must remain at ready angles')
    up=reader_frame(robot,q,config)[:3,:3]@np.asarray(cfg['up_axis'])
    tilt=np.rad2deg(np.arccos(np.clip(up[2],-1.,1.)))
    if tilt>cfg['max_tilt_deg' if endpoint else 'max_path_tilt_deg']:
        raise ValueError('scanner upright tilt limit')
    return float(tilt)


def solve_fixed_wrist(robot,start,label,config,canceled=lambda:False):
    cfg=config['barcode_fixed_wrist'];label=pose(label);q=vector(start,28).copy()
    # The arm already carries the reader in this posture; do not silently move
    # a displaced wrist back while presenting this as a locked-wrist path.
    check_posture(robot,q,config)
    q[4:7]=cfg['joint_values'];seed=q[:4].copy();near,far=cfg['distance_range_m']
    def frame(arm):
        trial=q.copy();trial[:4]=arm
        return trial,reader_frame(robot,trial,config)
    def residual(arm):
        if canceled():raise InterruptedError('reader planning canceled')
        _,scanner=frame(arm);delta=label[:3,3]-scanner[:3,3];distance=np.linalg.norm(delta)
        direction=delta/max(distance,1e-9);up=scanner[:3,:3]@np.asarray(cfg['up_axis'])
        return np.r_[scanner[:3,2]-direction,up-[0,0,1],
            10*max(cfg['preferred_min_distance_m']-distance,0),10*max(distance-far,0),.0001*(arm-seed)]
    for initial in [seed,np.asarray(config['home_joints'])[:4],np.mean(robot.limits[:4],axis=1)]:
        robot.ik_calls+=1
        sol=least_squares(residual,np.clip(initial,robot.limits[:4,0]+1e-8,robot.limits[:4,1]-1e-8),bounds=(robot.limits[:4,0],robot.limits[:4,1]),max_nfev=180)
        trial,scanner=frame(sol.x);delta=label[:3,3]-scanner[:3,3];distance=np.linalg.norm(delta)
        direction=delta/max(distance,1e-9);aim=np.rad2deg(np.arccos(np.clip(scanner[:3,2]@direction,-1,1)))
        toward=-direction;front=toward@label[:3,2]
        pitch=np.rad2deg(np.arctan2(toward@label[:3,1],front));skew=np.rad2deg(np.arctan2(toward@label[:3,0],front))
        if not near<=distance<=far or aim>cfg['max_aim_deg'] or front<=0 or max(abs(pitch),abs(skew))>cfg['max_view_deg']:continue
        try:
            check_posture(robot,trial,config,command=True,endpoint=True)
            robot.clearance(trial,config['obstacles'],config['collision_margin'],config['attached_radius'])
        except ValueError:continue
        return trial
    raise ValueError('barcode unreachable with ready wrist locked and scanner upright')
