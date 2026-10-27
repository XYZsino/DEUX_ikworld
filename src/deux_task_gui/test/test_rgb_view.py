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
from deux_task_gui.rgb_view import render_rgb,corners_for


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


def test_selected_surface_box_projects_into_its_image_and_handles_mm_depth():
    o,d=fixture();pixels,extent=corners_for(o,d,'selected')
    assert pixels.shape==(8,2) and np.all(extent>0)
    assert pixels[:,0].min()>15 and pixels[:,0].max()<65
    assert pixels[:,1].min()>15 and pixels[:,1].max()<55
    jpg,meta=render_rgb(o,d,'selected');im=np.array(PILImage.open(BytesIO(jpg)))
    assert meta['box_valid'] and ((im[:,:,1]>140)&(im[:,:,0]<150)).sum()>40
    depth=np.frombuffer(bytes(o.depth.data),dtype='<f4').reshape(72,96)
    o.depth.encoding='16UC1';o.depth.step=192;o.depth.data=(depth*1000).astype('<u2').tobytes()
    pixels_mm,_=corners_for(o,d,'selected')
    np.testing.assert_allclose(pixels,pixels_mm,atol=.15)


@pytest.mark.parametrize('case',['different_target','different_sequence','different_stamp','different_frame','missing_tf','wrong_tf_time','missing_mask','no_depth'])
def test_invalid_observation_shows_rgb_without_a_stale_or_guessed_box(case):
    o,d=fixture();target='selected'
    if case=='different_target':target='another'
    if case=='different_sequence':d.sequence+=1
    if case=='different_stamp':d.header=deepcopy(d.header);d.header.stamp.sec+=1
    if case=='different_frame':d.header=deepcopy(d.header);d.header.frame_id='other_camera'
    if case=='missing_tf':o.camera_transform_valid=False
    if case=='wrong_tf_time':o.base_from_camera.header.stamp.sec=11
    if case=='missing_mask':d.boxes[0].polygon_xy=[]
    if case=='no_depth':o.depth.data=np.full((72,96),np.nan,dtype='<f4').tobytes()
    jpg,m=render_rgb(o,d,target)
    assert not m['box_valid'] and m['reason'] and PILImage.open(BytesIO(jpg)).size==(96,72)


def test_depth_spike_does_not_expand_selected_box():
    o,d=fixture();_,before=corners_for(o,d,'selected')
    z=np.frombuffer(bytes(o.depth.data),dtype='<f4').copy().reshape(72,96);z[30,30]=2.7;o.depth.data=z.tobytes()
    _,after=corners_for(o,d,'selected');np.testing.assert_allclose(after,before,atol=.002)


def test_selection_roi_is_visible_before_target_selection():
    o,d=fixture();d.selection_roi_valid=True;d.selection_roi_xyxy=[5.,5.,90.,65.]
    jpg,m=render_rgb(o,d,'')
    assert m['selection_roi_xyxy']==[5.,5.,90.,65.] and not m['box_valid']
    image=np.array(PILImage.open(BytesIO(jpg)))
    assert ((image[:,:,0]>180)&(image[:,:,1]>130)&(image[:,:,2]<150)).sum()>40
    d.sequence+=1
    _,m=render_rgb(o,d,'')
    assert m['selection_roi_xyxy'] is None
