from types import SimpleNamespace
import numpy as np
import pytest
from deux_task_core.placement import trajectory_clearance,skip_collision

@pytest.mark.parametrize('stage',['PlaceApproach','Place','Release','PlaceRetreat','Retreat'])
def test_skips_geometry_but_keeps_limits(stage):
 c=dict(perception_policy='head_rgbd',placement=dict(enabled=True,fixed_joint_motion=True,skip_fixed_motion_collision_checks=True))
 def forbidden(*args):raise AssertionError('geometry check called')
 r=SimpleNamespace(limits=np.tile([-1.,1.],(28,1)),joint_limit_tolerance=0.,clearance=forbidden)
 assert trajectory_clearance(r,c,stage,np.zeros(28),0)==1e6
 q=np.zeros(28);q[14]=1.1
 with pytest.raises(ValueError,match='joint limit'):trajectory_clearance(r,c,stage,q,0)

@pytest.mark.parametrize('stage',['ClearReader','Approach','Descend','AlignBarcode','Prepare','Recover'])
def test_other_stages_still_check_geometry(stage):
 c=dict(perception_policy='head_rgbd',placement=dict(enabled=True,fixed_joint_motion=True,skip_fixed_motion_collision_checks=True),obstacles=[],collision_margin=.02)
 r=SimpleNamespace(clearance=lambda *args:.123)
 assert not skip_collision(c,stage)
 assert trajectory_clearance(r,c,stage,np.zeros(28),0)==.123

def test_opt_out_default_is_false():
 c=dict(perception_policy='head_rgbd',placement=dict(enabled=True,fixed_joint_motion=True))
 assert not skip_collision(c,'Place')
