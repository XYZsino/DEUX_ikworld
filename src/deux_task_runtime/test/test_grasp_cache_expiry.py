from types import SimpleNamespace
import pytest
from deux_task_core.planning import grasp_cache_max_age, validate_scene
from deux_task_core.synthetic import fixture
from deux_task_runtime.common import current, ros_time
from deux_task_runtime.execution_node import ExecutionNode
from deux_task_runtime.motion_planner_node import MotionPlannerNode


def config():
    return dict(perception_policy='head_rgbd', stationary_source_confirmed=True,
                moving_source=False, grasp_pose_max_age=20.)


def test_frozen_age_policy():
    c = config()
    assert not current(ros_time(1), 1000., grasp_cache_max_age(c))
    c['stationary_grasp_pose_expiry_disabled'] = True
    age = grasp_cache_max_age(c)
    assert current(ros_time(1), 1000., age)
    assert not current(ros_time(0), 1000., age)
    assert not current(ros_time(1001), 1000., age)
    node = SimpleNamespace(config=c)
    assert MotionPlannerNode.grasp_pose_age(node, 'Descend') == age
    assert MotionPlannerNode.grasp_pose_age(node, 'Prepare') is None
    node.now = lambda: 1000.
    node.frozen_grasp = lambda p: True
    ExecutionNode.grasp_age_guard(node, SimpleNamespace(target_acquired_at=ros_time(1)))
    with pytest.raises(ValueError, match='expired'):
        ExecutionNode.grasp_age_guard(node, SimpleNamespace(target_acquired_at=ros_time(0)))


@pytest.mark.parametrize('change', [dict(moving_source=True),
    dict(stationary_source_confirmed=False), dict(perception_policy='other'),
    dict(stationary_grasp_pose_expiry_disabled='true')])
def test_unlimited_requires_stationary_configuration(tmp_path, change):
    c = fixture(tmp_path)
    c.update(config(), stationary_grasp_pose_expiry_disabled=True)
    c.update(change)
    with pytest.raises(ValueError, match='stationary|unlimited'):
        validate_scene(c)
