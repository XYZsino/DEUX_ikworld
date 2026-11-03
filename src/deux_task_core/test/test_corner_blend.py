import numpy as np
import pytest
from deux_task_core.planning import hermite


def test_reversing_joints_do_not_all_stop_at_internal_knot():
    knots=np.array([[0.,0.],[.6,.5],[.2,.4]])
    v=np.array([2.,2.]);a=np.array([3.,3.]);dt=.002
    speeds=[]
    for blend in (0.,.6):
        t,q=hermite(knots,v,a,dt,corner_blend=blend)
        idx=np.argmin(np.linalg.norm(q-knots[1],axis=1))
        speeds.append(np.linalg.norm((q[idx+1]-q[idx-1])/(2*dt)))
        np.testing.assert_allclose(q[[0,-1]],knots[[0,-1]],atol=1e-10)
        velocity=np.diff(q,axis=0)/dt
        assert np.max(abs(velocity)/v)<=1.001
        assert np.max(abs(np.diff(velocity,axis=0)/dt)/a)<=1.001
        assert np.linalg.norm(velocity[0])<.01
        assert np.linalg.norm(velocity[-1])<.01
    assert speeds[0]<.01
    assert speeds[1]>.05


@pytest.mark.parametrize('value',[-1.,1.1,float('nan')])
def test_invalid_corner_blend_rejected(value):
    with pytest.raises(ValueError,match='corner_blend'):
        hermite([[0.],[1.]],[2.],[3.],.02,corner_blend=value)


def test_default_matches_explicit_zero_blend():
    knots=[[0.,0.],[.4,.5],[.1,.6]]
    a=hermite(knots,np.ones(2),np.ones(2)*3,.02)
    b=hermite(knots,np.ones(2),np.ones(2)*3,.02,corner_blend=0)
    for x,y in zip(a,b):np.testing.assert_array_equal(x,y)
