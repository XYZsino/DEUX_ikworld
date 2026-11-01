"""Pure task-space policy for demonstration-informed conveyor tracking.
All positions are base-frame grasp origins, not pixel ROI or reachability claims.
"""
import numpy as np
from .conveyor import forecast


def validate_policy(p):
    lo=np.asarray(p['xy_min_m'],float);hi=np.asarray(p['xy_max_m'],float)
    if lo.shape!=(2,) or hi.shape!=(2,) or not np.isfinite([lo,hi]).all() or np.any(lo>=hi):
        raise ValueError('invalid tracking XY workspace')
    if not 0 <= p['entry_margin_m'] < np.min(hi-lo)/2:raise ValueError('invalid entry margin')
    if not lo[1]<=p['standby_y_m']<=hi[1]:raise ValueError('standby Y outside workspace')
    for k in ('ready_fraction','pregrasp_fraction'):
        if not np.isfinite(p[k]) or not 0<=p[k]<=1:raise ValueError('invalid hand fraction')
    for k in ('close_duration_s','close_xy_m','close_height_m','descent_speed_m_s'):
        if not np.isfinite(p[k]) or p[k]<=0:raise ValueError('invalid '+k)
    if p['close_height_m']>.02 or p['descent_speed_m_s']>.1:raise ValueError('invalid overlap policy')
    descent_xy=p.get('descent_xy_m',.03)
    if not np.isfinite(descent_xy) or not 0<descent_xy<=.12:raise ValueError('invalid descent XY tolerance')
    hold=p.get('prediction_hold_s',.5)
    if not np.isfinite(hold) or not .5<=hold<=1.:raise ValueError('invalid prediction hold duration')
    standby=p.get('standby_right_arm_rad')
    if standby is not None and (np.asarray(standby).shape!=(7,) or not np.isfinite(standby).all()):raise ValueError('invalid standby joints')
    return p


def inside_xy(p,points,margin=0.):
    a=np.asarray(points,float)
    return bool(a.size and a.shape[-1]>=2 and np.isfinite(a).all() and
        np.all(a[...,:2]>=np.asarray(p['xy_min_m'])+margin-1e-9) and
        np.all(a[...,:2]<=np.asarray(p['xy_max_m'])-margin+1e-9))


def forecast_inside(p,obj,v,grasp,start,end,margin=0.):
    # Straight translation, constant orientation: convex XY box needs only endpoints.
    return inside_xy(p,[(forecast(obj,v,t)@grasp)[:3,3] for t in (start,end)],margin)


def follow_admitted(p,obj,v,grasp,t,previous=False):
    margin=0. if previous else p['entry_margin_m']
    return forecast_inside(p,obj,v,grasp,t,t,margin)


def constrained_point(point,linear,initial,p,follow):
    point=np.array(point,float).copy();linear=np.array(linear,float).copy()
    if not follow:
        point[0]=np.clip(point[0],p['xy_min_m'][0],p['xy_max_m'][0])
        if point[0]<=p['xy_min_m'][0] or point[0]>=p['xy_max_m'][0]:linear[0]=0.
        point[1]=p['standby_y_m'];point[2]=initial[2,3]
        linear[1:3]=0.
    return point,linear


def closure_ok(p,xy,height,remaining,inside,observation_ok):
    return bool(np.isfinite([xy,height,remaining]).all() and 0<=xy<=p['close_xy_m']
                and 0<=height<=p['close_height_m'] and remaining>=p['close_duration_s']+.05
                and inside and observation_ok)


def workspace_descent_ready(p,xy,elapsed,approach_s,follow,inside,roi_ok):
    """Admit gradual descent from measured alignment, without a meeting schedule."""
    return bool(np.isfinite([xy,elapsed,approach_s]).all()
                and 0<=xy<=p.get('descent_xy_m',.03) and elapsed>=approach_s
                and follow and inside and roi_ok)


def entry_prediction(p,obj,v,stamp,now):
    """Future object origin at wall-clock now+lookahead, including image latency."""
    lookahead=float(p['entry_prediction_s'])
    if not np.isfinite([stamp,now,lookahead]).all() or now<stamp or lookahead!=1.:
        raise ValueError('invalid entry prediction time')
    point=forecast(obj,v,now-stamp+lookahead)[:3,3]
    return inside_xy(p,[point]),point
