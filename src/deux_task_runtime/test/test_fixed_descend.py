import numpy as np
import pytest
from deux_task_core.synthetic import fixture
from deux_task_core.planning import Planner
from deux_task_runtime.ik_diagnostic import report


def test_fixed_drop_uses_measured_base_z_and_matches_preview(tmp_path, monkeypatch):
    c=fixture(tmp_path)
    c.update(approach_height=.15,descend_distance_m=.05)
    planner=Planner(c)
    q=np.asarray(c['home_joints'])
    current=planner.robot.fk(q)[planner.robot.ee['right']]
    target=current.copy();target[2,3]-=.05
    captured=[]
    def capture(side, goal, seed):
        captured.append(goal.copy())
        raise RuntimeError('captured IK goal')
    monkeypatch.setattr(planner.robot,'ik',capture)
    with pytest.raises(RuntimeError,match='captured IK goal'):
        planner.plan('Descend',q,np.eye(4))
    np.testing.assert_allclose(captured[0],target)
    d=report(planner.robot,c,'test','Descend',q,np.eye(4),'test')
    np.testing.assert_allclose(d['target_pose'],target)
