from types import SimpleNamespace as NS
import numpy as np
import pytest
from deux_task_perception.core import (normalize_depth, normalize_rgb, validate_bundle,
                                      History, joint_values, validate_objects, JOINT_NAMES)


def header(t=1_000_000_000, frame='external_optical'):
    return NS(stamp=NS(sec=t//10**9, nanosec=t % 10**9), frame_id=frame)


def image(values, encoding, pad=0):
    a = np.asarray(values)
    h, w = a.shape[:2]
    row = a[0].nbytes
    data = b''.join(a[y].tobytes()+b'\x00'*pad for y in range(h))
    return NS(header=header(), width=w, height=h, encoding=encoding,
              step=row+pad, data=data, is_bigendian=a.dtype.byteorder == '>')


def info(w=640, h=480):
    return NS(header=header(), width=w, height=h, k=[500.,0,320,0,500,240,0,0,1],
              p=[500.,0,320,0,0,500,240,0,0,0,1,0], binning_x=0, binning_y=0,
              roi=NS(x_offset=0,y_offset=0,width=0,height=0))


def test_zed_bgra_padding_rgb_conversion():
    msg = image(np.array([[[1,2,3,255],[4,5,6,255]]], dtype='u1'), 'bgra8', 8)
    assert normalize_rgb(msg).tolist() == [[[3,2,1],[6,5,4]]]


@pytest.mark.parametrize('encoding', ['mono16', '16UC1'])
def test_zed_openni_big_endian_mm(encoding):
    msg = image(np.array([[1000,0],[2500,500]], dtype='>u2'), encoding, 4)
    a, valid = normalize_depth(msg)
    assert a[0,0] == 1 and a[1,0] == 2.5 and np.isnan(a[0,1])
    assert valid == .75 and a.dtype == np.dtype('<f4')


def test_float_depth_invalid_values_not_zero_or_finite_fill():
    a, valid = normalize_depth(image(np.array([[1,0,-1,np.inf,np.nan]], 'f4'), '32FC1'))
    assert valid == .2 and np.isnan(a[0,1:]).all()


def bundle():
    return [image(np.zeros((480,640,3),'u1'),'rgb8'),
            image(np.ones((480,640),'f4'),'32FC1'), info(), info()]


def test_bundle_native_size_and_matching_calibration():
    validate_bundle(*bundle(),640,480,20_000_000)


@pytest.mark.parametrize('failure', ['size','frame','calibration','stamp','roi','binning','uncalibrated'])
def test_reject_unsafe_bundle(failure):
    b = bundle()
    if failure == 'size': b[0].width = 1280
    if failure == 'frame': b[1].header.frame_id = 'head_optical'
    if failure == 'calibration': b[3].p[0] += 2
    if failure == 'stamp': b[1].header = header(1_050_000_000)
    if failure == 'roi': b[2].roi.x_offset = 10
    if failure == 'binning': b[2].binning_x = 2
    if failure == 'uncalibrated': b[2].k[0] = 0
    with pytest.raises(ValueError): validate_bundle(*b,640,480,20_000_000)


def test_truncated_image_rejected():
    m = image(np.ones((2,3),'f4'),'32FC1')
    m.data = m.data[:-1]
    with pytest.raises(ValueError): normalize_depth(m)


def test_history_never_uses_future_or_stale_measurement():
    h = History()
    for stamp in (100,300,200): h.add(NS(header=header(stamp)))
    assert h.at(250,60).header.stamp.nanosec == 200
    assert h.at(250,40) is None
    assert h.at(90,100) is None
    h.clear()
    assert h.at(400,1000) is None


def test_joint_order_missing_duplicate_and_nonfinite():
    names = list(reversed(JOINT_NAMES))
    m = NS(name=names, position=list(range(28)))
    assert joint_values(m,JOINT_NAMES) == list(reversed(range(28)))
    m.position[0] = np.nan
    with pytest.raises(ValueError): joint_values(m,JOINT_NAMES)
    m = NS(name=['a','a'],position=[0.,1.])
    with pytest.raises(ValueError): joint_values(m,['a'])
    with pytest.raises(KeyError): joint_values(NS(name=[],position=[]),['a'])


def test_base_pose_and_covariance_validation():
    pose = NS(position=NS(x=0.,y=0.,z=1.),orientation=NS(x=0.,y=0.,z=0.,w=1.))
    d = NS(track_id='bag-1', pose=NS(pose=pose,covariance=[0.]*36),confidence=.9)
    m = NS(header=header(frame='base_link'),detections=[d])
    validate_objects(m,'base_link')
    with pytest.raises(ValueError): validate_objects(m,'head')
    d.pose.covariance[0] = -1
    with pytest.raises(ValueError): validate_objects(m,'base_link')
    d.pose.covariance[0] = 0
    pose.orientation.w = 0
    with pytest.raises(ValueError): validate_objects(m,'base_link')
