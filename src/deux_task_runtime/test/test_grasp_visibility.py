from types import SimpleNamespace as NS
import pytest
from deux_task_core.planning import validate_scene
from deux_task_core.synthetic import fixture
from deux_task_runtime.head_task_node import HeadTaskNode
from deux_task_runtime.task_node import TaskNode
from deux_task_runtime.common import ros_time

@pytest.mark.parametrize('stage',['Approach','Descend','Close'])
def test_visibility_can_be_disabled_without_disabling_parent_guard(monkeypatch,stage):
    calls=[];monkeypatch.setattr(TaskNode,'guard',lambda *args:calls.append(True))
    n=NS(config={'moving_source':False,'grasp_pose_max_age':20.,'max_observation_age':1.},state=stage,target_id='bag',now=lambda:10.,
         grasp_objects=NS(header=NS(stamp=ros_time(9.)),detections=[NS(track_id='bag',pose=NS(pose=NS(position=NS(x=1.,y=0.,z=0.))))]),
         observation=NS(right_ee_valid=True,base_from_right_ee=NS(header=NS(stamp=ros_time(9.5)),transform=NS(translation=NS(x=0.,y=0.,z=0.)))),visible=lambda:False)
    values=vars(n);n=object.__new__(HeadTaskNode);n.__dict__.update(values)
    with pytest.raises(ValueError,match='target lost'):HeadTaskNode.guard(n,None,None)
    n.config['stationary_grasp_visibility_check_enabled']=False
    HeadTaskNode.guard(n,None,None);assert len(calls)==2
    n.grasp_objects.header.stamp=ros_time(0)
    with pytest.raises(ValueError,match='expired'):HeadTaskNode.guard(n,None,None)

@pytest.mark.parametrize('change',[{'moving_source':True},{'stationary_source_confirmed':False},{'perception_policy':'other'},{'stationary_grasp_visibility_check_enabled':'false'}])
def test_disabled_check_requires_stationary_configuration(tmp_path,change):
    c=fixture(tmp_path);c.update(perception_policy='head_rgbd',moving_source=False,stationary_source_confirmed=True,stationary_grasp_visibility_check_enabled=False);c.update(change)
    with pytest.raises(ValueError,match='visibility'):validate_scene(c)
