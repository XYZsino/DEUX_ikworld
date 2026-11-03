from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from deux_task_core.axis_grasp_trial import z_directions, select
from deux_task_core.planning import Planner


def test_only_opposite_object_z_directions():
    obj=Rotation.from_euler('xyz',[.2,.3,.4]).as_matrix()
    grasp=Rotation.from_euler('xyz',[.6,.7,.8]).as_matrix()
    choices=z_directions(obj,grasp)
    assert [y for y,_ in choices]==[0,180]
    for yaw,rot in choices:
        np.testing.assert_allclose(rot@grasp.T@np.array([0,0,1]),obj[:,2],atol=1e-12)
        assert np.linalg.det(rot)==pytest.approx(1)


@pytest.mark.parametrize('failures', [0,1,2])
def test_home_order_lazy_fallback_and_full_path_checks(monkeypatch,failures):
    c=dict(temporary_axis_grasp_trial=True,descend_distance_m=.06,approach_height=.16,
           home_joints=[0.]*28,grasp_object_from_right_ee=np.eye(4).tolist())
    home=np.eye(4);home[:3,:3]=Rotation.from_euler('z',170,degrees=True).as_matrix()
    r=SimpleNamespace(ee_fk=lambda *args:home,ik_calls=0)
    calls=[];seen=[];count=[0]
    def plan(self,stage,start,obj,canceled):
        calls.append(stage)
        assert not self.config['temporary_axis_grasp_trial']
        assert self.config['_axis_trial_hold_descent']
        if stage=='Approach':seen.append(np.array(self.config['grasp_object_from_right_ee']))
        if stage=='Descend':
            count[0]+=1
            if count[0]<=failures:raise ValueError('collision clearance')
        return dict(path=np.zeros((2,28)))
    monkeypatch.setattr(Planner,'plan',plan)
    p=SimpleNamespace(config=c,robot=r)
    if failures==2:
        with pytest.raises(ValueError,match='neither 0 nor 180'):select(p,np.zeros(28),np.eye(4),lambda:False)
    else:
        data=select(p,np.zeros(28),np.eye(4),lambda:False)
        assert data['axis_grasp_report']['selected_yaw_deg']==(180 if failures==0 else 0)
    assert calls==['Approach','Descend']*min(failures+1,2)
    np.testing.assert_allclose(seen[0][:3,:3],Rotation.from_euler('z',180,degrees=True).as_matrix())
    assert c['temporary_axis_grasp_trial'] and '_axis_trial_hold_descent' not in c
