"""Short-horizon descent targets and position-only close qualification."""
import numpy as np

CLOSE_POSITION_TOLERANCE_M = .05
from scipy.interpolate import BPoly
from deux_task_core.conveyor import forecast
from deux_task_core.position_first_preview import desired

def height_state(profile,t,default_height):
    if profile is None:return float(default_height),0.
    a=profile['start'];b=a+profile['duration']
    if t<=a:return profile['height0'],profile['velocity0']
    if t>=b:return profile['height1'],0.
    if profile.get('kind')=='scheduled_trapezoid':
        if profile['height0']-profile['height1']<=1e-10:return profile['height1'],0.
        u=t-a;v=profile['speed'];r=profile['ramp_s'];d=profile['duration']
        if u<r:return profile['height0']-.5*v*u*u/r,-v*u/r
        if u<=d-r:return profile['height0']-v*(u-.5*r),-v
        left=d-u
        return profile['height1']+.5*v*left*left/r,-v*left/r
    curve=BPoly.from_derivatives([a,b],[[profile['height0'],profile['velocity0'],0.],[profile['height1'],0.,0.]])
    return float(curve(t)),float(curve.derivative()(t))

def next_profile(old,splice,duration,default_height,admitted,speed=.06):
    previous=old.get('descent_profile')
    if not admitted:return previous
    h,hd=height_state(previous,splice,default_height)
    if not np.isfinite([h,hd,speed]).all() or h<0 or not 0<speed<=.1:
        raise ValueError('invalid gradual descent height/speed')
    end=max(0.,h-speed*duration)
    # A long stop interval near zero can make a quintic dip below zero then
    # reverse. Shorten the height ramp (not the joint path horizon) to brake
    # monotonically, preserving the incoming height and vertical velocity.
    if hd<0 and end==0.:
        duration=min(duration,2.*h/(-hd))
        if duration<=0:raise ValueError('nonzero downward velocity at zero height')
    result=dict(start=float(splice),duration=float(duration),height0=h,velocity0=hd,height1=end)
    curve=BPoly.from_derivatives([splice,splice+duration],[[h,hd,0.],[end,0.,0.]])
    values=curve(np.linspace(splice,splice+duration,51))
    if values.min()<-1e-8 or values.max()>h+1e-8 or np.max(np.diff(values))>1e-8:
        raise ValueError('gradual descent profile is not monotonic')
    return result

def target(initial,obj,v,grasp,height,approach,t,profile=None,lookahead=1.,workspace=None,workspace_follow=True,hold_rotation=None):
    lead_rate=0.
    if profile is not None and t>=profile['start'] and lookahead>0:
        remaining,rate=height_state(profile,t,height)
        u=float(np.clip(remaining/max(height,1e-9),0.,1.))
        lead_rate=lookahead*6*u*(1-u)*rate/max(height,1e-9)
        lookahead*=u*u*(3-2*u)
    pose,feed=desired(initial,obj,v,grasp,height,approach,10.,t,lookahead=lookahead,workspace=workspace,workspace_follow=workspace_follow)
    if t>approach:feed[:3]+=np.asarray(v[:3])*lead_rate
    if profile is not None and t>=profile['start']:
        h,hd=height_state(profile,t,height)
        point=forecast(obj,v,t)@grasp
        pose[2,3]=point[2,3]+h
        feed[2]=v[2]+hd
    if hold_rotation is not None:
        pose[:3,:3]=hold_rotation;feed[3:]=0.
    return pose,feed

def position_errors(robot,q,obj,v,stamp,at,grasp):
    if at<stamp:raise ValueError('prediction timestamp is in future')
    actual=robot.ee_fk('right',q)
    wanted=forecast(obj,v,at-stamp)@np.asarray(grasp)
    delta=actual[:3,3]-wanted[:3,3]
    return float(np.linalg.norm(delta[:2])),float(np.linalg.norm(delta)),actual,wanted

def close_eligible(distance,elapsed,brake_start,inside_roi,observation_ok):
    return bool(np.isfinite(distance) and distance<=CLOSE_POSITION_TOLERANCE_M and elapsed>=0
                and brake_start-elapsed>=.55 and inside_roi and observation_ok)
