import copy,json,sys,time
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path('tools').resolve()))
from reusable_intercept import admit,signature

@pytest.fixture
def case():
 record=json.loads(Path('calibration/reusable_intercept.json').read_text())
 inp=json.loads(Path('.setup/moving-upper-trial/latest-intercept-input.json').read_text())
 c=inp['config'];c['tracking_grasp_policy']=json.loads(Path('calibration/tracking_grasp_policy.json').read_text())
 plan=record['plan'];q=np.array(plan['path'][0]);v=np.array(record['nominal_velocity']);obj=np.array(plan['object_at_start']);obj[:3,3]-=v[:3]*2
 return c,record,q,obj,v,plan['geometry']

def test_reuses_exact_path_and_fixed_point(case):
 c,r,q,o,v,g=case;a=admit(c,r,q,o,v,0.,g)
 b=admit(c,r,q,o,v,.1,g)
 assert a['path']==b['path']==r['plan']['path']
 assert a['fixed_interception']['pose']==b['fixed_interception']['pose']
 assert b['fixed_interception']['nominal_contact_s']==pytest.approx(a['fixed_interception']['nominal_contact_s']-.1)

def test_config_invalidates(case):
 c,r,q,o,v,g=case;c=copy.deepcopy(c);c['collision_margin']+=.01
 with pytest.raises(ValueError,match='configuration changed'):admit(c,r,q,o,v,0.,g)

def test_start_pose_invalidates(case):
 c,r,q,o,v,g=case;q[16]+=.03
 with pytest.raises(ValueError,match='ready pose'):admit(c,r,q,o,v,0.,g)

def test_rotation_and_height_guard(case):
 from scipy.spatial.transform import Rotation
 c,r,q,o,v,g=case;turned=o.copy();turned[:3,:3]=Rotation.from_euler('z',30,degrees=True).as_matrix()@turned[:3,:3]
 with pytest.raises(ValueError,match='orientation'):admit(c,r,q,turned,v,0.,g)
 o[2,3]+=.03
 with pytest.raises(ValueError,match='height'):admit(c,r,q,o,v,0.,g)

def test_missed_arrival_rejected(case):
 c,r,q,o,v,g=case
 with pytest.raises(ValueError,match='no longer reachable'):admit(c,r,q,o,v,4.,g)

def test_belt_change_rejected(case):
 c,r,q,o,v,g=case;v[1]+=.05
 with pytest.raises(ValueError,match='speed'):admit(c,r,q,o,v,0.,g)
