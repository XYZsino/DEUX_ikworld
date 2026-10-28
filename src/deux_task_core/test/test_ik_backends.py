import xml.etree.ElementTree as ET
import numpy as np
import pytest
from deux_task_core.synthetic import fixture
from deux_task_core.model import RobotModel, JOINT_NAMES
from deux_task_core.planning import Planner, scene_version
from deux_task_core.ik_backends import CheckedIK, backend_name, write_arm_urdf, arm_names
from deux_task_core.configuration import load_configuration
import yaml


class FixedAnswer(CheckedIK):
    def make_engine(self,side,path):return None
    def solve_arm(self,engine,target,seed,continuity_weight):return self.answer


def test_default_backend_and_selection_are_part_of_scene_identity(tmp_path):
    config=fixture(tmp_path)
    assert backend_name(config)=='scipy'
    assert not hasattr(Planner(config).robot,'ik_solver')
    version=scene_version(config)
    config['ik_backend']='pyroki'
    assert scene_version(config)!=version
    config['ik_backend']='typo'
    with pytest.raises(ValueError,match='ik_backend'):Planner(config)


def test_real_default_is_pyroki_and_explicit_selection_still_wins():
    assert backend_name({})=='pyroki'
    assert backend_name({'synthetic':False})=='pyroki'
    for selected in ('scipy','pyroki','curobo'):
        assert backend_name({'synthetic':False,'ik_backend':selected})==selected


def test_omitted_backend_is_resolved_before_configuration_versioning(tmp_path):
    config=fixture(tmp_path);config.pop('ik_backend')
    path=tmp_path/'installation.yaml';path.write_text(yaml.safe_dump(config))
    loaded,version=load_configuration(path)
    assert loaded['ik_backend']=='pyroki'
    config['ik_backend']='pyroki'
    assert version==scene_version(config)
    path.write_text(yaml.safe_dump(config))
    assert load_configuration(path)[1]==version


@pytest.mark.parametrize('side',['left','right'])
def test_checked_solver_preserves_all_other_joints_and_rejects_invalid_answers(tmp_path,side):
    robot=RobotModel(fixture(tmp_path)['robot']);solver=FixedAnswer(robot)
    q=np.linspace(-.1,.1,28);indices=[JOINT_NAMES.index(n) for n in arm_names(side)]
    target=robot.ee_fk(side,q)
    solver.answer=q[indices].copy()
    np.testing.assert_array_equal(solver.solve(side,target,q,.02),q)
    solver.answer=np.full(7,np.nan)
    with pytest.raises(ValueError,match='finite'):solver.solve(side,target,q,0.)
    solver.answer=q[indices].copy();solver.answer[0]=5.
    with pytest.raises(ValueError,match='joint limit'):solver.solve(side,target,q,0.)
    solver.answer=q[indices].copy();solver.answer[0]+=.2
    with pytest.raises(ValueError,match='residual'):solver.solve(side,target,q,0.)
    with pytest.raises(ValueError,match='continuity'):solver.solve(side,target,q,-1.)


def test_chain_export_preserves_configured_fixed_prismatic_offset(tmp_path):
    config=fixture(tmp_path);path=config['robot']['urdf_path']
    root=ET.parse(path).getroot()
    ET.SubElement(root,'link',name='lift_link')
    joint=ET.SubElement(root,'joint',name='lift_joint',type='prismatic')
    ET.SubElement(joint,'parent',link='base_link');ET.SubElement(joint,'child',link='lift_link')
    ET.SubElement(joint,'axis',xyz='0 0 1')
    ET.SubElement(joint,'origin',xyz='0.1 0 0.2')
    for first in root.findall('joint'):
        if first.get('name') in ('arm_l_joint1','arm_r_joint1'):first.find('parent').set('link','lift_link')
    ET.ElementTree(root).write(path)
    config['robot']['fixed_joints']={'lift_joint':.35}
    robot=RobotModel(config['robot']);out=tmp_path/'arm.urdf'
    write_arm_urdf(robot,'right',out)
    chain=ET.parse(out).getroot()
    lift=next(j for j in chain.findall('joint') if j.get('name')=='lift_joint')
    assert lift.get('type')=='fixed'
    np.testing.assert_allclose(list(map(float,lift.find('origin').get('xyz').split())),[.1,0,.55])
    assert {j.get('name') for j in chain.findall('joint') if j.get('type')!='fixed'}==set(arm_names('right'))


def test_missing_optional_dependency_never_falls_back(tmp_path,monkeypatch):
    import builtins
    original=builtins.__import__
    def blocked(name,*args,**kwargs):
        if name=='ik_pyroki':raise ImportError('test missing dependency')
        return original(name,*args,**kwargs)
    monkeypatch.setattr(builtins,'__import__',blocked)
    config=fixture(tmp_path);config['ik_backend']='pyroki'
    with pytest.raises(ValueError,match='dependencies unavailable'):Planner(config)


def test_batch_preserves_other_joints_and_reports_per_target_errors(tmp_path):
    class BatchAnswers(FixedAnswer):
        def solve_arm_batch(self,side,targets,seeds,continuity_weight):
            result=seeds.copy();result[1,0]+=1.
            return result,['']*len(seeds)
    robot=RobotModel(fixture(tmp_path)['robot']);solver=BatchAnswers(robot)
    q=np.zeros((2,28));q[:,7:14]=.1
    target=np.array([robot.ee_fk('right',row) for row in q])
    solved,errors=solver.solve_batch('right',target,q)
    assert errors[0]=='' and 'residual' in errors[1]
    np.testing.assert_array_equal(solved[0],q[0])
    np.testing.assert_array_equal(solved[1],q[1])
    assert robot.ik_calls==2
