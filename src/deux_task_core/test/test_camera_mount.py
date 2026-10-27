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
