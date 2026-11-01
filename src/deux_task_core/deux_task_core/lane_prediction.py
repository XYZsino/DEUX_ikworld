"""Preview-only belt motion: tangent direction, scalar speed, fixed plane height."""
from collections import deque
import numpy as np

class LanePrediction:
    def __init__(self, plane):
        plane=np.asarray(plane,float)
        if plane.shape!=(4,) or not np.isfinite(plane).all() or np.linalg.norm(plane[:3])<1e-9:raise ValueError('invalid support plane')
        self.normal=plane[:3]/np.linalg.norm(plane[:3]);self.offset=plane[3]/np.linalg.norm(plane[:3]);self.tracks={}

    @staticmethod
    def slope(t,p):
        slopes=[(p[j]-p[i])/(t[j]-t[i]) for i in range(len(t)) for j in range(i+1,len(t)) if t[j]-t[i]>=.2]
        return np.median(slopes,axis=0) if slopes else np.zeros(3)

    def update(self, identity, stamp, position):
        p=np.asarray(position,float);n=self.normal
        if not np.isfinite(p).all():return None
        self.tracks={k:v for k,v in self.tracks.items() if stamp-v[-1][0]<=.75}
        history=self.tracks.setdefault(identity,deque())
        if history and stamp<=history[-1][0]:return None
        history.append((stamp,p.copy()))
        while history and stamp-history[0][0]>3.:history.popleft()
        if len(history)<6 or stamp-history[0][0]<.7:return None
        t=np.array([x[0]-stamp for x in history]);points=np.array([x[1] for x in history])
        tangent=points-(points@n)[:,None]*n
        vector=self.slope(t,tangent);norm=np.linalg.norm(vector)
        recent=t>=-1.2
        tt=t[recent];pp=points[recent]
        if len(tt)<6 or np.ptp(tt)<.7:return None
        height=float(np.median(pp@n));height_scatter=float(np.std(pp@n))
        # Large vertical changes are not compatible with the resting-bag model.
        if abs(float(p@n)-height)>.03:return None
        if norm<.005:
            velocity=np.zeros(3);anchor=np.median(pp,axis=0);direction=np.zeros(3);speed=0.
        else:
            direction=vector/norm
            values=pp@direction
            speed=float(self.slope(tt,values));velocity=direction*speed
            if abs(speed)>.3:return None
            # Robust line intercept at acquisition time. Fix normal and lateral position.
            along=float(np.median(values-speed*tt));lateral=np.median(pp-values[:,None]*direction,axis=0)
            anchor=lateral+direction*along
        anchor=anchor+n*(height-anchor@n)
        return dict(position=anchor.tolist(),velocity=velocity.tolist(),direction=direction.tolist(),speed_m_s=speed,height_m=height+self.offset,height_std_m=height_scatter,observations=len(tt),window_s=float(np.ptp(tt)))
