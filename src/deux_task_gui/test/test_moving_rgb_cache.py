from collections import deque
from types import SimpleNamespace
import time
from deux_task_gui.rgb_view import RgbPreview
from test_rgb_view import fixture
from deux_task_msgs.msg import ObjectDetectionArray,ObjectDetection

def test_moving_trial_clears_old_icp_after_tracking_ends():
 obs,_=fixture();pose=ObjectDetectionArray();pose.header.frame_id='base_link';pose.header.stamp=obs.header.stamp
 d=ObjectDetection(track_id='old');d.pose.pose.position.z=1.;d.pose.pose.orientation.w=1.;pose.detections=[d]
 node=SimpleNamespace(status={'workflow':'MovingUpperTrial','target_id':'old'},icp_preview={'tracking':False,'target_id':''},perception_only=lambda:False,get_parameter=lambda name:SimpleNamespace(value='base_link'),has_parameter=lambda name:False)
 rgb=RgbPreview.__new__(RgbPreview);rgb.node=node;rgb.observations=deque([(time.monotonic(),obs)]);rgb.objects=deque([(time.monotonic(),pose)]);rgb.tracks=deque();rgb.cached_pose=pose;rgb.cached_target='old';rgb.key=None
 rgb.update()
 assert rgb.cached_pose is None and rgb.cached_target=='' and not rgb.objects
 assert not rgb.metadata['icp_axes_valid'] and rgb.metadata['target_id']==''
 assert rgb.snapshot() is not None


def test_static_scan_clears_pregrasp_icp():
 obs,_=fixture();pose=ObjectDetectionArray();pose.header.frame_id='base_link';pose.header.stamp=obs.header.stamp
 d=ObjectDetection(track_id='old');d.pose.pose.position.z=1.;d.pose.pose.orientation.w=1.;pose.detections=[d]
 node=SimpleNamespace(status={'workflow':'head_rgbd_pick_scan_place','target_id':'old','state':'TrackBarcode'},perception_only=lambda:False,get_parameter=lambda name:SimpleNamespace(value='base_link'),has_parameter=lambda name:False)
 rgb=RgbPreview.__new__(RgbPreview);rgb.node=node;rgb.observations=deque([(time.monotonic(),obs)]);rgb.objects=deque([(time.monotonic(),pose)]);rgb.tracks=deque();rgb.cached_pose=pose;rgb.cached_target='old';rgb.key=None
 rgb.update()
 assert rgb.cached_pose is None and not rgb.objects
 assert not rgb.metadata['icp_axes_valid']
 assert rgb.snapshot() is not None
