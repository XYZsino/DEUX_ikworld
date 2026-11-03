import numpy as np
import pytest
from test_fixed_placement import configured
from deux_task_core.placement import validate

def demo(tmp_path):
 c=configured(tmp_path);p=c['placement'];p.update(source='demonstration_temporary_operator_confirmed',allow_uncalibrated_demonstration=True)
 p['hover_right_pose'][0][3]+=.1
 return c

def test_confirmed_fixed_demo_accepts_diagonal_route(tmp_path):
 validate(demo(tmp_path))

def test_demo_cartesian_path_still_requires_vertical_route(tmp_path):
 c=demo(tmp_path);c['placement']['fixed_joint_motion']=False
 with pytest.raises(ValueError,match='vertical'):validate(c)

def test_unconfirmed_demo_rejected_on_real_robot(tmp_path):
 c=demo(tmp_path);c['synthetic']=False;c['placement']['allow_uncalibrated_demonstration']=False
 with pytest.raises(ValueError,match='calibrated'):validate(c)

def test_demo_still_requires_destination_bounds(tmp_path):
 c=demo(tmp_path);c['placement']['release_right_pose'][0][3]+=100
 with pytest.raises(ValueError,match='destination'):validate(c)
