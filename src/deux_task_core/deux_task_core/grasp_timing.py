"""Common finger timing for screening and executable conveyor trajectories."""
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from .model import JOINT_NAMES,HAND_NAMES


def knots(durations, overlap):
    ends=np.cumsum(durations)
    close_start=ends[1]-durations[1]*overlap
    times=np.unique(np.r_[0.,close_start,*[
        np.linspace(a,b,max(3,int(np.ceil((b-a)/.12))+1))[1:]
        for a,b in zip(np.r_[0.,ends[:-1]],ends)]])
    return times,close_start


def initial_durations(config,robot):
    cfg=config['conveyor_tracking']
    if 'fixed_grasp_durations_s' in cfg:
        return np.asarray(cfg['fixed_grasp_durations_s'],dtype=float)
    d=np.array([cfg['durations'][s] for s in ('Track','Descend','Close')],dtype=float)
    overlap=d[1]*cfg.get('close_descent_overlap',0.)
    # Close in config describes the complete finger motion, including the part
    # overlapping descent. The last phase contains only the remaining time.
    d[2]=max(.05,d[2]-overlap)
    hands=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    opened=np.asarray(config.get('hand_pregrasp',config['hand_open']))
    closed=np.asarray(config['hand_closed'])
    limit=cfg.get('grasp_planning_ratio',.98)
    for _ in range(8):
        kt,begin=knots(d,cfg.get('close_descent_overlap',0.))
        total=float(sum(d));u=np.clip((kt-begin)/(total-begin),0.,1.)
        b=10*u**3-15*u**4+6*u**5
        q=opened[None]+b[:,None]*(closed-opened)[None]
        slopes=np.gradient(q,kt,axis=0);slopes[kt<=begin+1e-9]=0.;slopes[-1]=0.
        ts=np.linspace(0,total,int(np.ceil(total/config['sample_period']))+1)
        path=CubicHermiteSpline(kt,q,slopes)(ts)
        v=np.diff(path,axis=0)/np.diff(ts)[:,None]
        a=np.diff(np.vstack([np.zeros(len(hands)),v,np.zeros(len(hands))]),axis=0)/np.r_[np.diff(ts),np.diff(ts)[-1]][:,None]
        factor=max(np.max(abs(v)/robot.velocity[hands]),np.sqrt(np.max(abs(a)/robot.acceleration[hands])))
        if factor<=limit:return d
        d[2]=(d[2]+overlap)*max(1.03,float(factor)/limit*1.03)-overlap
    raise ValueError('finger closure timing did not converge')
