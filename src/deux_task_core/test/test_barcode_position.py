from types import SimpleNamespace
import numpy as np
import pytest
from deux_task_core.barcode_position import reader_look_at,label_quality

@pytest.mark.parametrize('point',[[1,2,3],[0,0,-1],[0,0,1]])
def test_look_at_and_standoff(point):
    current=np.eye(4);p=np.array(point,dtype=float)
    t=reader_look_at(current,p,.12)
    np.testing.assert_allclose(t[:3,2],p/np.linalg.norm(p),atol=1e-9)
    assert np.linalg.norm(p-t[:3,3]) == pytest.approx(.12)
    assert np.linalg.det(t[:3,:3]) == pytest.approx(1)

def test_position_not_misrepresented_as_surface():
    m=SimpleNamespace(orientation_valid=False,plane_rmse_m=float('nan'),valid_fraction=1/9)
    assert label_quality(m,{'barcode_position_only':True})
    assert not label_quality(m,{})
    m.valid_fraction=0
    assert not label_quality(m,{'barcode_position_only':True})


def test_barcode_aim_offset_is_in_base_not_reader_frame():
    from scipy.spatial.transform import Rotation
    current=np.eye(4)
    current[:3,:3]=Rotation.from_euler('y',90,degrees=True).as_matrix()
    current[:3,3]=[.1,.2,.4]
    barcode=np.array([.6,.3,.7]);saved=barcode.copy()
    aim=barcode+np.array([0,0,.1])
    target=reader_look_at(current,barcode,.12,[0,0,.1])
    np.testing.assert_allclose(target[:3,3]+.12*target[:3,2],aim,atol=1e-10)
    np.testing.assert_array_equal(barcode,saved)
    assert np.linalg.norm(aim-target[:3,3])==pytest.approx(.12)


def test_reader_right_yaw_preserves_position():
    current=np.eye(4)
    original=reader_look_at(current,[1.,0.,0.],.12)
    rotated=reader_look_at(current,[1.,0.,0.],.12,yaw_offset_deg=-30.)
    np.testing.assert_allclose(rotated[:3,3],original[:3,3])
    np.testing.assert_allclose(rotated[:3,2],[np.sqrt(3)/2,-.5,0.],atol=1e-10)
