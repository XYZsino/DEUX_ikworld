from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import pytest
import yaml
from deux_task_core.camera_mount import append_zed2i, mesh_vertices, mount_transform
from deux_task_core.model import origin

ASSETS=Path(__file__).resolve().parents[2]/'deux_task_bringup'


def test_camera_gap_optical_direction_and_stereo_baseline():
    config=yaml.safe_load((ASSETS/'config/zed2i_mount.yaml').read_text())
    mesh=ASSETS/'meshes/zed2i.stl'
    transform,audit=mount_transform(config,mesh)
    points=(mesh_vertices(mesh)+[-.01,0,.015])@transform[:3,:3].T+transform[:3,3]
    assert points[:,2].min()-config['eye_display_top_xyz'][2]==pytest.approx(.003)
    assert audit['calibration_confirmed'] is False
    root=ET.fromstring('<robot name="test"><link name="lift"/></robot>')
    append_zed2i(root,config,mesh)
    fk={'lift':np.eye(4)}
    for joint in root.findall('joint'):
        fk[joint.find('child').get('link')]=fk[joint.find('parent').get('link')]@origin(joint.find('origin'))
    left=fk['zed_left_camera_frame_optical'];right=fk['zed_right_camera_frame_optical']
    np.testing.assert_allclose(left[:3,2],[2**-.5,0,-2**-.5],atol=1e-12)
    assert np.linalg.norm(left[:3,3]-right[:3,3])==pytest.approx(.12)
    with pytest.raises(ValueError,match='already present'):append_zed2i(root,config,mesh)


def test_mount_requires_explicit_reference():
    config=yaml.safe_load((ASSETS/'config/zed2i_mount.yaml').read_text())
    mesh=ASSETS/'meshes/zed2i.stl'
    config['eye_display_top_xyz'][0]=float('nan')
    with pytest.raises(ValueError,match='dimensions'):mount_transform(config,mesh)


def test_committed_robot_has_connected_zed_and_portable_meshes():
    root=ET.parse(ASSETS/'urdf/deux_fixed_zed2i.urdf').getroot()
    links=[link.get('name') for link in root.findall('link')]
    assert len(links)==len(set(links))
    joints=root.findall('joint');parents={}
    for joint in joints:
        parent=joint.find('parent').get('link');child=joint.find('child').get('link')
        assert parent in links and child in links and child not in parents
        parents[child]=parent
    roots=set(links)-set(parents);assert len(roots)==1
    for link in links:
        seen=set()
        while link in parents:
            assert link not in seen
            seen.add(link);link=parents[link]
        assert link in roots
    lift=root.find("joint[@name='lift_joint']")
    assert lift.get('type')=='fixed'
    np.testing.assert_allclose(origin(lift.find('origin'))[:3,3],[.114,0,.543164])
    config=yaml.safe_load((ASSETS/'config/zed2i_mount.yaml').read_text())
    expected=ET.fromstring('<robot name="camera"><link name="lift"/></robot>')
    append_zed2i(expected,config,ASSETS/'meshes/zed2i.stl')
    for joint in expected.findall('joint'):
        actual=root.find("joint[@name='%s']"%joint.get('name'))
        assert actual is not None
        assert actual.find('parent').get('link')==joint.find('parent').get('link')
        np.testing.assert_allclose(origin(actual.find('origin')),origin(joint.find('origin')),atol=1e-12)
    for mesh in root.iter('mesh'):
        uri=mesh.get('filename')
        assert uri.startswith(('package://deux_description/','package://deux_task_bringup/'))
        if uri.startswith('package://deux_task_bringup/'):
            assert (ASSETS/uri.removeprefix('package://deux_task_bringup/')).is_file()
    assert root.find('mujoco') is None
