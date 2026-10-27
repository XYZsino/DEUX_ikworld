from types import SimpleNamespace
import numpy as np
import pytest
from deux_task_sim.grip_physics import apply_grip_physics


def fixture():
    names=['right_hand_thumb_1','right_hand_index_2','left_hand_index_2','conveyor','bread_1']
    model=SimpleNamespace(ngeom=5,geom_bodyid=np.arange(5),body=lambda i:SimpleNamespace(name=names[i]),
        actuator_gainprm=np.tile([6.,0.,0.],(4,1)),actuator_biasprm=np.tile([0.,-6.,-.6],(4,1)),
        actuator_forcerange=np.tile([-5.,5.],(4,1)),geom_friction=np.array([[3.,.01,.001],[3.,.01,.001],[3.,.01,.001],[.001,.005,.0001],[.8,.01,.001]]))
    return SimpleNamespace(model=model,_hand_u={'right':np.array([1,2])})


def test_grip_experiment_preserves_belt_object_and_left_hand_properties():
    sim=fixture();mu=sim.model.geom_friction.copy();gains=sim.model.actuator_gainprm.copy()
    config=dict(right_hand_torque_limit_nm=10.,right_hand_stiffness_scale=2.,finger_sliding_friction_scale=1.5)
    audit=apply_grip_physics(sim,config)
    np.testing.assert_array_equal(sim.model.geom_friction[2:],mu[2:])
    np.testing.assert_array_equal(sim.model.actuator_gainprm[[0,3]],gains[[0,3]])
    assert audit['applied_sliding_friction']==[4.5]
    state=sim.model.geom_friction.copy();apply_grip_physics(sim,config)
    np.testing.assert_array_equal(sim.model.geom_friction,state)
    apply_grip_physics(sim)
    np.testing.assert_array_equal(sim.model.geom_friction,mu)
    np.testing.assert_array_equal(sim.model.actuator_gainprm,gains)


@pytest.mark.parametrize('value',[float('nan'),-1.,0.])
def test_invalid_sim_grip_parameters_rejected(value):
    with pytest.raises(ValueError):apply_grip_physics(fixture(),{'right_hand_torque_limit_nm':value})
