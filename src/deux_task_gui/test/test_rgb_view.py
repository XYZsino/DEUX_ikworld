from copy import deepcopy
from io import BytesIO
import numpy as np
from PIL import Image as PILImage
import pytest
pytest.importorskip('rclpy')
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from deux_task_msgs.msg import Observation,DetectionArray,BoundingBox
from deux_task_gui.rgb_view import render_rgb


def fixture():
    w,h=96,72;head=Header(stamp=Time(sec=12),frame_id='zed_optical')
    rgb=np.full((h,w,3),60,dtype=np.uint8)
    y,x=np.mgrid[:h,:w];depth=(.8+x*.001+y*.0003).astype('<f4')
    k=[80.,0.,48.,0.,80.,36.,0.,0.,1.];p=[80.,0.,48.,0.,0.,80.,36.,0.,0.,0.,1.,0.]
    info=CameraInfo(header=head,width=w,height=h,k=k,p=p)
    obs=Observation(header=head,sequence=7,
        rgb=Image(header=head,width=w,height=h,encoding='rgb8',step=w*3,data=rgb.tobytes()),
        depth=Image(header=head,width=w,height=h,encoding='32FC1',step=w*4,data=depth.tobytes()),
        rgb_info=info,depth_info=deepcopy(info),camera_transform_valid=True)
    obs.base_from_camera.header=Header(stamp=Time(sec=12),frame_id='base_link')
    obs.base_from_camera.child_frame_id='zed_optical';obs.base_from_camera.transform.rotation.w=1.
    box=BoundingBox(class_name='bread_bag',track_id='selected',confidence=.9,xyxy=[20.,20.,60.,50.],polygon_xy=[20.,20.,60.,20.,60.,50.,20.,50.])
    det=DetectionArray(header=head,sequence=7,boxes=[box])
    return obs,det


def test_icp_axes_follow_pose_rotation_and_require_same_acquisition():
    from deux_task_msgs.msg import ObjectDetectionArray,ObjectDetection
    from deux_task_gui.rgb_view import icp_axes_for
    o,d=fixture();obj=ObjectDetection(track_id='selected')
    obj.pose.pose.position.z=1.
    obj.pose.pose.orientation.z=2**-.5;obj.pose.pose.orientation.w=2**-.5
    poses=ObjectDetectionArray(header=Header(stamp=Time(sec=12),frame_id='base_link'),sequence=7,detections=[obj])
    pixels=icp_axes_for(o,poses,'selected')
    assert pixels[1,1]>pixels[0,1] and abs(pixels[1,0]-pixels[0,0])<1e-6
    assert pixels[2,0]<pixels[0,0]
    _,meta=render_rgb(o,d,'selected',objects=poses)
    assert meta['icp_axes_valid']
    poses.sequence+=1
    _,meta=render_rgb(o,d,'selected',objects=poses)
    assert not meta['icp_axes_valid'] and not meta['box_valid']


def test_direct_bundle_retains_original_buffers_and_stable_acquisition_id():
    from deux_task_perception.camera_input import CameraInput
    o,_=fixture()
    args=(o.rgb,o.depth,o.rgb_info,o.depth_info,96,72,.02)
    a=CameraInput.bundle(*args);b=CameraInput.bundle(*args)
    assert a.rgb is o.rgb and a.depth is o.depth and a.rgb_info is o.rgb_info
    assert a.sequence==b.sequence==12_000_000_000
    bad=deepcopy(o.depth_info);bad.header.stamp.nanosec=30_000_000
    with pytest.raises(ValueError):CameraInput.bundle(o.rgb,o.depth,o.rgb_info,bad,96,72,.02)


def test_detection_boxes_and_roi_are_not_drawn():
    o,d=fixture();d.selection_roi_valid=True;d.selection_roi_xyxy=[5.,5.,90.,65.]
    with_boxes,meta=render_rgb(o,d,'selected')
    without_boxes,_=render_rgb(o,None,'selected')
    assert with_boxes==without_boxes
    assert not meta['box_valid'] and meta['selection_roi_xyxy'] is None

def test_historical_axes_use_current_camera_tf_and_preserve_pose_stamp():
    from deux_task_msgs.msg import ObjectDetectionArray,ObjectDetection
    from deux_task_gui.rgb_view import icp_axes_for
    o,d=fixture()
    obj=ObjectDetection(track_id='selected')
    obj.pose.pose.position.z=1.;obj.pose.pose.orientation.w=1.
    poses=ObjectDetectionArray(header=Header(stamp=Time(sec=12),frame_id='base_link'),sequence=7,detections=[obj])
    original=icp_axes_for(o,poses,'selected')
    o.header.stamp.sec=30;o.sequence=8
    o.base_from_camera.header.stamp.sec=30
    o.base_from_camera.transform.translation.x=.1
    _,meta=render_rgb(o,d,'selected',objects=poses,allow_historical=True)
    assert meta['icp_axes_valid'] and meta['icp_axes_held']
    assert meta['icp_pose_age_s']==18. and poses.header.stamp.sec==12
    moved=icp_axes_for(o,poses,'selected',allow_historical=True)
    assert moved[0,0]<original[0,0]
    _,strict=render_rgb(o,d,'selected',objects=poses)
    assert not strict['icp_axes_valid']
    o.camera_transform_valid=False
    _,invalid=render_rgb(o,d,'selected',objects=poses,allow_historical=True)
    assert not invalid['icp_axes_valid']
