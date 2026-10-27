import numpy as np
import pytest
from deux_task_core.motion_filter import MotionFilter


def test_moving_target_velocity_prediction_and_single_outlier_recovery():
    f=MotionFilter();var=[.003**2]*3+[.02**2]
    rng=np.random.default_rng(41)
    for i in range(20):
        t=1+i*.1;z=np.array([.5,.08*(t-1),.7,0.])+rng.normal(0,[.001,.001,.001,.005])
        state,status=f.update('bag',t,i,z,var)
        assert status=='accepted'
    assert abs(state.state[5]-.08)<.007
    saved=state.state.copy();stamp=state.stamp
    rejected,status=f.update('bag',3.,20,[.9,-.5,.7,0.],var)
    assert rejected is None and status=='jump'
    assert f.tracks['bag'].stamp==stamp
    np.testing.assert_equal(f.tracks['bag'].state,saved)
    state,status=f.update('bag',3.1,21,[.5,.168,.7,0.],var)
    assert status=='accepted'
    prediction,_=f.project(state,3.3)
    assert abs(prediction[1]-.184)<.003
    with pytest.raises(ValueError):f.project(state,4.)


def test_yaw_wrap_duplicate_and_reacquisition_require_new_samples():
    f=MotionFilter();v=[.0001]*4
    s,_=f.update('a',1.,1,[0,0,0,np.pi-.01],v)
    s,status=f.update('a',1.1,2,[0,0,0,-np.pi+.01],v)
    assert status=='accepted' and abs(s.state[3]-np.pi)<.03
    assert f.update('a',1.1,3,[0,0,0,0],v)[1]=='out_of_order'
    assert f.update('a',1.2,2,[0,0,0,0],v)[1]=='out_of_order'
    new,_=f.update('a',2.,4,[0,0,0,0],v)
    assert new.observations==1 and np.linalg.norm(new.state[4:])==0
    with pytest.raises(ValueError):f.update('a',2.1,5,[np.nan,0,0,0],v)


def test_known_belt_velocity_is_only_an_initial_prior():
    f=MotionFilter(initial_velocity=[0,-.08,0,0],initial_velocity_std=.01)
    variance=[.003**2]*3+[.02**2]
    state,_=f.update('bag',1.,1,[.5,0,.7,0],variance)
    assert state.state[5]==pytest.approx(-.08)
    # Observations of a slower object override the initial belt estimate.
    for i in range(1,31):
        state,status=f.update('bag',1+i*.1,i+1,[.5,-.05*i*.1,.7,0],variance)
        assert status=='accepted'
    assert abs(state.state[5]+.05)<.004
    with pytest.raises(ValueError):MotionFilter(initial_velocity=[0,-2,0,0])
