import numpy as np
from deux_task_core.plan_preflight import compare_prediction
from deux_task_core.conveyor import forecast


def test_matching_motion_reuses_and_changed_position_or_velocity_researches():
    c=dict(max_target_drift=.04,max_target_angle_rad=.15)
    obj=np.eye(4);v=[0,-.09,0,0];latest=forecast(obj,v,1.)
    assert compare_prediction(c,obj,v,10.,latest,v,11.,15.)['reuse']
    changed=latest.copy();changed[0,3]+=.05
    assert not compare_prediction(c,obj,v,10.,changed,v,11.,15.)['reuse']
    assert not compare_prediction(c,obj,v,10.,latest,[0,-.12,0,0],11.,15.)['reuse']
