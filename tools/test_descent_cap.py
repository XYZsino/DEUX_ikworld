import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path('tools').resolve()))
from scheduled_descent import profile_for,finish_time
from progressive_descent import height_state

def test_drop_is_capped_and_floor_not_reported_as_contact():
 p=profile_for(.08,1.,.06,.05)
 states=np.array([height_state(p,t,.08) for t in np.linspace(1.,1.+p['duration'],301)])
 assert np.isclose(states[0,0]-states[-1,0],.05)
 assert np.isclose(states[-1,0],.03)
 assert np.max(np.diff(states[:,0]))<=1e-10
 assert states[:,0].min()>=.03-1e-10
 assert states[:,1].min()>=-.06-1e-10

def test_five_cm_profile_and_zero_distance():
 p=profile_for(.05,0.,.06,.05)
 assert np.isclose(p['duration'],.05/.06+.15)
 assert height_state(p,p['duration'],.05)==(0.,0.)
 p=profile_for(.08,0.,.06,0.)
 assert height_state(p,.0005,.08)==(.08,0.)
