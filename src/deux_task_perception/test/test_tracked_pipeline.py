"""ROS contracts for selecting image tracks and stopping ICP after grasp."""
import time
import threading
import numpy as np
import pytest
rclpy=pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from deux_task_msgs.msg import Observation,DetectionArray,DetectionFrame,PerceptionRequest,VisualEvidence,VisualEvidenceArray,BarcodeTarget
from deux_task_perception.tracked_detection_node import TrackedDetectionNode


class ImageDetector:
    identity='test-image-detector'
    def track(self,rgb,stamp):
        assert rgb.shape==(48,64,3)
        return [dict(track_id='image-bag',class_id=0,class_name='bread_bag',confidence=.95,
                     xyxy=[8.,6.,58.,44.],polygon_xy=[8.,6.,58.,6.,58.,44.,8.,44.]),
                dict(track_id='image-code',class_id=1,class_name='barcode',confidence=.95,
                     xyxy=[20.,15.,45.,35.],polygon_xy=[20.,15.,45.,15.,45.,35.,20.,35.])]


@pytest.mark.parametrize("integrated", [False, True])
def test_image_track_selected_icp_then_visual_only(integrated):
    rclpy.init(domain_id=176);ex=SingleThreadedExecutor();source=rclpy.create_node('image_source')
    params=[Parameter('width',value=64),Parameter('height',value=48)]
    if integrated:
        from deux_task_perception.tracked_perception_node import TrackedPerceptionNode
        v,u=np.mgrid[7:44,9:58]
        model=np.column_stack(((u.ravel()-32)*.01,(v.ravel()-24)*.01,np.zeros(u.size)))
        node=TrackedPerceptionNode(detector=ImageDetector(),
            model_data=(model,np.tile([0.,0.,-1.],(len(model),1)),'plane'),
            parameter_overrides=params+[Parameter('reference_frame',value='base_link'),
                Parameter('icp_initial_angles',value=1)])
    else:
        node=TrackedDetectionNode(detector=ImageDetector(),parameter_overrides=params)
    ex.add_node(source);ex.add_node(node)
    frames=[];detections=[];visual=[];labels=[];candidates=[];poses=[]
    from deux_task_msgs.msg import ObjectDetectionArray
    qos=qos_profile_sensor_data
    subs=[source.create_subscription(t,topic,values.append,qos) for t,topic,values in
        [(ObjectDetectionArray,'/task/object_detections',poses),(DetectionFrame,'/task/detection_frames',frames),(DetectionArray,'/task/detections',detections),
         (VisualEvidenceArray,'/task/visual_candidates',candidates),(VisualEvidence,'/task/visual_evidence',visual),(BarcodeTarget,'/task/barcode_target',labels)]]
    images=source.create_publisher(Observation,'/task/observation',qos)
    requests=source.create_publisher(PerceptionRequest,'/task/perception_request',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    tf=StaticTransformBroadcaster(source)
    transform=TransformStamped(header=Header(frame_id='base_link'),child_frame_id='head_optical')
    transform.transform.rotation.w=1.;tf.sendTransform(transform)
    release = threading.Event()
    sequence=0
    def publish():
        nonlocal sequence
        h=Header(stamp=source.get_clock().now().to_msg(),frame_id='head_optical')
        rgb=Image(header=h,height=48,width=64,encoding='rgb8',step=192,data=np.zeros((48,64,3),np.uint8).tobytes())
        depth=Image(header=h,height=48,width=64,encoding='32FC1',step=256,data=np.ones((48,64),np.float32).tobytes())
        info=CameraInfo(header=h,height=48,width=64,p=[100.,0.,32.,0.,0.,100.,24.,0.,0.,0.,1.,0.],k=[100.,0.,32.,0.,100.,24.,0.,0.,1.],r=np.eye(3).ravel().tolist())
        images.publish(Observation(header=h,sequence=sequence,rgb=rgb,depth=depth,rgb_info=info,depth_info=info));sequence+=1
    timer=source.create_timer(.04,publish)
    def spin(predicate):
        end=time.monotonic()+5
        while not predicate():
            assert time.monotonic()<end,node.status
            ex.spin_once(timeout_sec=.01)
    def request(mode,identity):
        now=source.get_clock().now().to_msg()
        requests.publish(PerceptionRequest(header=Header(stamp=now),request_id=identity,target_id='image-bag',mode=mode,
            expires_at=Time(sec=now.sec+20,nanosec=now.nanosec)))
    try:
        spin(lambda:len(detections)>2);assert not frames
        request('locate','before-grasp');spin(lambda:len(poses if integrated else frames)>1)
        if integrated:
            assert not frames and not source.get_publishers_info_by_topic('/task/detection_frames')
            assert all(p.request_id=='before-grasp' and p.detections[0].track_id=='image-bag' for p in poses)
        assert all(len(f.boxes)==1 and f.boxes[0].track_id=='image-bag' and f.request_id=='before-grasp' for f in frames)
        if integrated:
            entered=threading.Event()
            original=node.estimator.estimate
            def slow_fit(payload):
                entered.set()
                assert release.wait(5)
                return original(payload)
            node.estimator.estimate=slow_fit
            spin(entered.is_set)
            count=node.accepted
            spin(lambda:node.accepted>=count+4)
            assert node.icp_worker.pending is not None  # Detection keeps running during slow ICP.
            assert node.icp_worker.dropped>0
            rejected=node.icp_rejected
        request('barcode','after-grasp');spin(lambda:len(labels)>1)
        if integrated:
            assert node.icp_worker.pending is None
            release.set()
            spin(lambda:node.icp_rejected>rejected)
            assert all(p.request_id=='before-grasp' for p in poses)
        assert labels[-1].target_id=='image-bag' and labels[-1].barcode_track_id=='image-code'
        assert visual[-1].bag_visible and visual[-1].barcode_visible and visual[-1].surface_center_valid
        assert not any(f.request_id=='after-grasp' for f in frames)
        assert detections[-1].boxes[0].track_id=='image-bag'
        spin(lambda:bool(candidates))
        observed=candidates[-1]
        assert observed.request_id=='after-grasp' and len(observed.candidates)==1
        assert observed.candidates[0].target_id=='image-bag'
        assert observed.candidates[0].surface_center_valid
        assert observed.header.stamp==observed.candidates[0].header.stamp
    finally:
        release.set()
        ex.shutdown();node.destroy_node();source.destroy_node();rclpy.shutdown()
