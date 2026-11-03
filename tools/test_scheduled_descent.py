import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('tools').resolve()))
from scheduled_descent import profile_for,finish_time,readiness
from progressive_descent import height_state

def test_monotonic_peak_speed_and_overlap():
 p=profile_for(.068,0.,.06)
 states=np.array([height_state(p,t,.068) for t in np.linspace(0,p['duration'],301)])
 assert states[-1,0]==0 and states[-1,1]==0
 assert states[:,0].min()>=0 and np.max(np.diff(states[:,0]))<=1e-10
 assert np.min(states[:,1])>=-.06-1e-10
 remaining=finish_time(p,0.,.015,.5)
 assert p['duration']<=remaining<p['duration']+.5
 assert remaining<1.6
 assert finish_time(p,remaining+1.,.015,.5)==0

def test_descent_before_old_approach_timer_and_lateral_rejection():
 policy=dict(descent_speed_m_s=.06,close_height_m=.015,close_duration_s=.5,descent_start_height_m=.1)
 r=readiness(.068,.08,[0.,-.1,0.,0.],[.6,-.08,1.],[.6,0.,.93],policy,100.,elapsed=.8)
 assert r['due'] and np.isclose(r['arrival_s'],.8)
 r=readiness(.068,.11,[0.,-.1,0.,0.],[.69,-.05,1.],[.6,0.,.93],policy,100.,elapsed=.8)
 assert not r['due']
