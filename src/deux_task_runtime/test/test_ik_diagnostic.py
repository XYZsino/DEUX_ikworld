import json
import numpy as np
from types import SimpleNamespace
from deux_task_core.synthetic import fixture
from deux_task_core.model import RobotModel
from deux_task_core.ik_backends import CheckedIK
from deux_task_runtime.ik_diagnostic import report
from deux_task_runtime import motion_planner_node as module

def test_report_composes_object_grasp_and_base_z_offset(tmp_path):
    c=fixture(tmp_path);robot=RobotModel(c['robot']);q=np.asarray(c['home_joints'])
    obj=np.eye(4);obj[:3,3]=[.5,.1,.8]
    d=report(robot,c,'version','Approach',q,obj,'rejected')
    grasp=obj@np.asarray(c['grasp_object_from_right_ee']);goal=grasp.copy();goal[2,3]+=c['approach_height']
    np.testing.assert_allclose(d['grasp_pose'],grasp)
    np.testing.assert_allclose(d['target_pose'],goal)
    assert d['rejected'] is None
    json.dumps(d,allow_nan=False)

def test_failed_planning_returns_diagnostic_without_a_path(tmp_path,monkeypatch):
    c=fixture(tmp_path);robot=RobotModel(c['robot']);q=np.asarray(c['home_joints'])
    monkeypatch.setattr(module,'cached_planner',lambda *a:SimpleNamespace(robot=robot))
    def fail(*a):raise ValueError('IK failed')
    monkeypatch.setattr(module,'compute_plan',fail)
    result=module.compute(c,'version','Approach',q,np.eye(4))
    assert result['planning_error']=='IK failed' and 'ik_diagnostic' in result
    assert 'q' not in result and 't' not in result
