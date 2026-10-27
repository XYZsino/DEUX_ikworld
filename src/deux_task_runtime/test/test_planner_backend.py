from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from deux_task_msgs.msg import MotionPlan
from deux_task_runtime.execution_node import ExecutionNode
from deux_task_runtime.motion_planner_node import cached_planner
from deux_task_core.planning import scene_version
from deux_task_core.synthetic import fixture


def test_executor_rejects_other_backend_before_motion():
    node=SimpleNamespace(config={'ik_backend':'curobo'},check_model_version=lambda:None)
    with pytest.raises(ValueError,match='backend differs'):
        ExecutionNode.validate(node,SimpleNamespace(plan=MotionPlan(ik_backend='pyroki')))


def test_worker_reuses_model_but_invalidates_after_scene_change(tmp_path):
    config=fixture(tmp_path);version=scene_version(config)
    first=cached_planner(config,version)
    first.robot.ik_calls=20
    assert cached_planner(config,version) is first and first.robot.ik_calls==0
    config['lift_height']+=.01
    with pytest.raises(ValueError,match='URDF changed'):cached_planner(config,version)
    assert cached_planner(config,scene_version(config)) is not first
