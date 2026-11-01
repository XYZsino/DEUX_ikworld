import numpy as np
import pytest
from deux_task_core.planning import Planner, fixed_hand_path, validate_scene
from deux_task_core.synthetic import fixture

@pytest.mark.parametrize('stage,key',[('Close','hand_close_duration_s'),('Release','hand_open_duration_s')])
def test_requested_hand_duration_is_used_by_planner(tmp_path,stage,key):
    c=fixture(tmp_path);c[key]=.5
    q=np.zeros(28)
    if stage=='Release':
        from deux_task_core.model import HAND_NAMES,JOINT_NAMES
        q[[JOINT_NAMES.index(n) for n in HAND_NAMES]]=c['hand_closed']
    result=Planner(c).plan(stage,q,np.eye(4))
    assert result['times'][-1]==pytest.approx(.5)

@pytest.mark.parametrize('velocity,acceleration',[([.1],[100.]),([100.],[.1])])
def test_infeasible_duration_is_rejected_instead_of_accelerating_or_retiming(velocity,acceleration):
    with pytest.raises(ValueError,match='limits'):
        fixed_hand_path([[0.],[1.]],np.array(velocity),np.array(acceleration),.02,.5)

@pytest.mark.parametrize('value',[0.,-1.,float('nan')])
def test_invalid_duration_rejected(tmp_path,value):
    c=fixture(tmp_path);c['hand_close_duration_s']=value
    with pytest.raises(ValueError,match='duration'):validate_scene(c)

def test_moving_grasp_preserves_requested_duration_including_overlap():
    from types import SimpleNamespace
    from deux_task_core.grasp_timing import initial_durations
    robot=SimpleNamespace(velocity=np.full(28,100.),acceleration=np.full(28,100.))
    c=dict(hand_close_duration_s=.5,hand_open=[0.]*14,hand_closed=[.03]*14,
           sample_period=.02,conveyor_tracking=dict(
               durations=dict(Track=.2,Descend=1.,Close=2.),close_descent_overlap=.1))
    d=initial_durations(c,robot)
    assert d[2]+d[1]*.1==pytest.approx(.5)
    robot.acceleration[:]=.001
    with pytest.raises(ValueError,match='duration'):initial_durations(c,robot)
    c['conveyor_tracking']['fixed_grasp_durations_s']=[.2,1.,2.]
    with pytest.raises(ValueError,match='conflicts'):initial_durations(c,robot)
