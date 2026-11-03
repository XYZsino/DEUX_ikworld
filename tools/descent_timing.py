"""Timing from validated FK paths and timestamped joint feedback; no commands."""
import json,time
from pathlib import Path
from collections import deque
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from deux_task_core.conveyor import forecast
from progressive_descent import height_state
from scheduled_descent import profile_for,finish_time

def curve(plan):return CubicHermiteSpline(plan['times'],plan['path'],plan['velocities'])

def validated_timing(robot,plan,policy):
 profile=plan.get('descent_profile')
 if profile is None:return None
 start=profile['start'];end=plan['brake_start_s']
 if end<=start:return None
 ts=np.linspace(start,end,max(3,int(np.ceil((end-start)/.04))+1));qs=curve(plan)(ts)
 obj=np.asarray(plan['object_at_start']);v=np.asarray(plan['velocity']);g=np.asarray(plan['grasp'])
 heights=np.array([robot.ee_fk('right',q)[2,3]-(forecast(obj,v,float(t))@g)[2,3] for t,q in zip(ts,qs)])
 def crossing(level):
  ids=np.flatnonzero(heights<=level)
  if not len(ids):return None
  i=int(ids[0])
  if i==0:return float(ts[0])
  return float(ts[i-1]+(ts[i]-ts[i-1])*(heights[i-1]-level)/max(heights[i-1]-heights[i],1e-12))
 close=crossing(policy['close_height_m']);end_height=crossing(profile['height1']+.005)
 finish=max(close+policy['close_duration_s'],end_height) if close is not None and end_height is not None else None
 return dict(source='validated_joint_path_fk',sample_dt_s=float(ts[1]-ts[0]),start_s=start,validated_until_s=end,close_crossing_s=close,completion_s=finish,completion_within_validated_path=finish is not None and finish<=end,height_end_m=float(heights[-1]),pose_arm_rad=np.asarray(qs[0])[14:21].tolist())

def estimate_delay(spline,t,actual,max_delay=.5):
 # Align q(t-delay) to measured q at its acquisition stamp, not receive time.
 if t<max_delay:return None
 past=np.asarray(spline(t-max_delay))[14:21];present=np.asarray(spline(t))[14:21]
 if np.linalg.norm(present-past)<.03:return None
 delays=np.linspace(0,max_delay,51);pred=np.asarray(spline(t-delays))[:,14:21]
 errors=np.sqrt(np.mean((pred-np.asarray(actual)[14:21])**2,axis=1));i=int(np.argmin(errors))
 if errors[i]>.025 or i==len(delays)-1:return None
 return float(delays[i])

class FeedbackTiming:
 def __init__(self,path,version):
  self.path=Path(path);self.version=version;self.bins={};self.saved_counts={};self.last_save=0.;self.last_stamp=None;self.plan_id=None;self.samples=deque(maxlen=32)
  if self.path.exists():
   try:
    d=json.loads(self.path.read_text())
    if d.get('version')==version:self.bins=d.get('bins',{})
   except (ValueError,OSError):pass
 def key(self,q):return ','.join(str(int(x)) for x in np.round(np.asarray(q)/np.deg2rad(20.)))
 def update(self,plan,actual,stamp,now,height,policy):
  identity=(plan['start_at'],plan.get('generation',0),len(plan['times']))
  if identity!=self.plan_id:self.spline=curve(plan);self.plan_id=identity
  epoch=plan['start_at'];elapsed=now-epoch;t=stamp-epoch
  if getattr(self,'epoch',None)!=epoch:self.samples.clear();self.last_stamp=None;self.epoch=epoch
  timing=plan.get('descent_timing') or {};profile=plan.get('descent_profile')
  ref=timing.get('pose_arm_rad',np.asarray(plan['path'][0])[14:21]);key=self.key(ref)
  if profile and stamp!=self.last_stamp and profile['start']+.5<=t<=min(plan['brake_start_s'],profile['start']+profile['duration']):
   lag=estimate_delay(self.spline,t,actual)
   if lag is not None:
    bucket=self.bins.setdefault(key,[]);bucket.append(lag);del bucket[:-100]
  if stamp!=self.last_stamp:
   self.samples.append((stamp,float(height)));self.last_stamp=stamp
  bucket=self.bins.get(key,[]);delay=float(np.percentile(bucket,90)) if len(bucket)>=5 else 0.
  nominal=finish_time(profile,elapsed,policy['close_height_m'],policy['close_duration_s']) if profile else finish_time(profile_for(max(0.,height),0.,policy['descent_speed_m_s'],policy.get('max_descent_m')),0.,policy['close_height_m'],policy['close_duration_s'])
  completion=timing.get('completion_s')
  path_remaining=max(0.,completion-elapsed) if completion is not None else nominal
  speed=policy['descent_speed_m_s'];measured_speed=None
  if profile and len(self.samples)>=4:
   recent=[(t,h) for t,h in self.samples if stamp-t<=.4 and t-epoch>=profile['start']]
   if len(recent)>=4 and recent[-1][0]-recent[0][0]>=.15:
    slopes=[-(b[1]-a[1])/(b[0]-a[0]) for a,b in zip(recent,recent[1:]) if b[0]>a[0]]
    if slopes:
     measured_speed=float(np.median(slopes))
     speed=max(.005,min(speed,measured_speed))
  floor=profile['height1'] if profile else max(0.,height-policy.get('max_descent_m',height))
  gap=max(0.,height-floor)
  live=profile_for(gap,0.,speed)
  live_remaining=finish_time(live,0.,max(0.,policy['close_height_m']-floor),policy['close_duration_s']) if gap>1e-6 else 0.
  # Height already reflects lag: use max, not lag + live estimate (double count).
  remaining=max(path_remaining+delay,live_remaining)
  if bucket and (now-self.last_save>=1. or (len(bucket)>=5 and self.saved_counts.get(key,0)<5)):
   self.path.parent.mkdir(parents=True,exist_ok=True);tmp=self.path.with_suffix('.tmp');tmp.write_text(json.dumps(dict(version=self.version,bins=self.bins)));tmp.replace(self.path);self.last_save=now;self.saved_counts[key]=len(bucket)
  return dict(remaining_s=remaining,validated_path_remaining_s=path_remaining,full_duration_validated=bool(timing.get('completion_within_validated_path')),delay_s=delay,delay_samples=len(bucket),delay_measured=len(bucket)>=5,live_remaining_s=live_remaining,measured_z_speed_m_s=measured_speed,feedback_age_s=now-stamp,pose_key=key)
