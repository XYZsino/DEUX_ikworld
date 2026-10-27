"""Acquisition-time XYZ/yaw constant-velocity Kalman filter with outlier gates.

Rejected/duplicate measurements never refresh the accepted acquisition time.
Predictions are bounded and carry their original measurement time separately.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class MotionEstimate:
    stamp: float
    sequence: int
    state: np.ndarray
    covariance: np.ndarray
    observations: int


class MotionFilter:
    def __init__(self, max_gap=.6, max_jump=.08, max_speed=.3,
                 max_yaw_jump=.5, innovation_gate=16., acceleration_std=.015,
                 initial_velocity=None, initial_velocity_std=.01):
        values=[max_gap,max_jump,max_speed,max_yaw_jump,innovation_gate,acceleration_std]
        if not np.isfinite(values).all() or min(values)<=0:raise ValueError('positive finite filter limits required')
        self.max_gap,self.max_jump,self.max_speed=max_gap,max_jump,max_speed
        self.max_yaw_jump,self.innovation_gate,self.acceleration_std=max_yaw_jump,innovation_gate,acceleration_std
        self.initial_velocity=None if initial_velocity is None else np.asarray(initial_velocity,float)
        if self.initial_velocity is not None and (self.initial_velocity.shape!=(4,) or not np.isfinite(self.initial_velocity).all() or np.linalg.norm(self.initial_velocity[:3])>max_speed):raise ValueError('invalid conveyor velocity prior')
        if not np.isfinite(initial_velocity_std) or initial_velocity_std<=0:raise ValueError('invalid velocity prior uncertainty')
        self.initial_velocity_std=initial_velocity_std
        self.tracks={}

    def project(self, estimate, stamp):
        dt=float(stamp)-estimate.stamp
        if not np.isfinite(dt) or not 0<=dt<=self.max_gap:raise ValueError('prediction outside bounded horizon')
        f=np.eye(8);f[:4,4:]=np.eye(4)*dt
        g=np.vstack([np.eye(4)*dt*dt/2,np.eye(4)*dt])
        q=np.diag([self.acceleration_std**2]*3+[.15**2])
        return f@estimate.state, f@estimate.covariance@f.T+g@q@g.T

    def update(self, identity, stamp, sequence, measurement, variances):
        z=np.asarray(measurement,dtype=float);v=np.asarray(variances,dtype=float)
        if z.shape!=(4,) or v.shape!=(4,) or not np.isfinite(np.r_[z,v,stamp]).all() or min(v)<=0:
            raise ValueError('invalid pose/covariance measurement')
        old=self.tracks.get(identity)
        if old is not None and (stamp<=old.stamp or sequence<=old.sequence):return None,'out_of_order'
        if old is None or stamp-old.stamp>self.max_gap:
            x=np.r_[z,np.zeros(4) if self.initial_velocity is None else self.initial_velocity]
            velocity_var=[self.max_speed**2]*3+[1.] if self.initial_velocity is None else [self.initial_velocity_std**2]*3+[.15**2]
            p=np.diag(np.r_[v,velocity_var])
            result=MotionEstimate(float(stamp),int(sequence),x,p,1)
        else:
            x,p=self.project(old,stamp)
            innovation=z-x[:4];innovation[3]=np.arctan2(np.sin(innovation[3]),np.cos(innovation[3]))
            if np.linalg.norm(innovation[:3])>self.max_jump or abs(innovation[3])>self.max_yaw_jump:
                return None,'jump'
            s=p[:4,:4]+np.diag(v)
            if innovation@np.linalg.solve(s,innovation)>self.innovation_gate:return None,'innovation'
            k=np.linalg.solve(s,p[:,:4].T).T
            x=x+k@innovation
            if np.linalg.norm(x[4:7])>self.max_speed:return None,'speed'
            a=np.eye(8);a[:,:4]-=k
            p=a@p@a.T+k@np.diag(v)@k.T
            result=MotionEstimate(float(stamp),int(sequence),x,p,old.observations+1)
        self.tracks[identity]=result
        self.tracks={k:t for k,t in self.tracks.items() if stamp-t.stamp<=self.max_gap}
        return result,'accepted'
