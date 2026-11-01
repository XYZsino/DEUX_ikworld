"""Pure timing policy. No hardware or ROS dependencies."""
import numpy as np
from progressive_descent import height_state

def profile_for(height,start,speed,max_drop=None):
 if not np.isfinite([height,start,speed]).all() or height<0 or speed<=0:raise ValueError('invalid descent timing')
 distance=height if max_drop is None else min(height,float(max_drop))
 if not np.isfinite(distance) or distance<0:raise ValueError('invalid descent distance')
 ramp=min(.15,distance/max(speed,1e-9));duration=distance/speed+ramp
 return dict(kind='scheduled_trapezoid',start=float(start),duration=max(.001,duration),height0=float(height),height1=float(height-distance),velocity0=0.,speed=float(speed),ramp_s=float(ramp))

def finish_time(profile,at,close_height,close_s):
 start=profile['start'];end=start+profile['duration'];lo=start;hi=end
 for _ in range(32):
  mid=(lo+hi)/2
  if height_state(profile,mid,profile['height0'])[0]>close_height:lo=mid
  else:hi=mid
 return max(0.,max(end,hi+close_s)-at)

def readiness(height,xy,velocity,hand,target,policy,now,existing=None,elapsed=0.):
 profile=existing or profile_for(max(0.,height),elapsed,policy['descent_speed_m_s'],policy.get('max_descent_m'))
 remaining=finish_time(profile,elapsed,policy['close_height_m'],policy['close_duration_s'])
 v=np.asarray(velocity[:2]);speed=np.linalg.norm(v)
 delta=np.asarray(hand[:2])-np.asarray(target[:2])
 arrival=float(delta@v/(speed*speed)) if speed>.005 else 0.
 lateral=float(np.linalg.norm(delta-v*arrival)) if speed>.005 else float(xy)
 # Arrival estimate decides timing; actual path/ROI/workspace/closure gates still apply.
 due=existing is not None or (height<=policy.get('descent_start_height_m',.10) and xy<=.12 and lateral<=.06 and arrival<=remaining)
 return dict(due=bool(due),remaining_s=float(remaining),arrival_s=arrival,lateral_m=lateral,contact_at=now+max(0.,arrival),profile=profile)
