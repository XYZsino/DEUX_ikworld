"""Synthetic detector, real yaw ICP and ROS transport; no YOLO/hardware claim."""
import time
from concurrent.futures import Future
from copy import deepcopy
import numpy as np
import pytest
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('deux_task_msgs.msg')
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from deux_task_msgs.msg import Observation, DetectionFrame, ObjectDetectionArray
from deux_task_perception.detection_node import DetectionNode
from deux_task_perception.icp_node import IcpNode
from deux_task_perception.core import validate_objects
from deux_task_perception.pipeline_node import PipelineNode


class SyntheticDetector:
    identity = 'synthetic-test-only'
    def __call__(self, rgb):
        assert rgb.shape == (12, 20, 3)
        assert rgb[0, 0].tolist() == [10, 20, 30]
        return [(2, 's1', .9, [0., 0., 19., 11.])]


def parameters(**values):
    return [Parameter(k, value=v) for k, v in values.items()]


def observation(node, sequence):
    msg = Observation()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = 'synthetic_optical'
    msg.sequence = sequence
    for image in (msg.rgb, msg.depth):
        image.header = msg.header
        image.width, image.height = 20, 12
    msg.rgb.encoding, msg.rgb.step, msg.rgb.data = 'rgb8', 60, bytes([10, 20, 30])*240
    msg.depth.encoding, msg.depth.step = '32FC1', 80
    msg.depth.data = np.full((12, 20), .5, dtype='<f4').tobytes()
    for info in (msg.rgb_info, msg.depth_info):
        info.header = msg.header
        info.width, info.height = 20, 12
        info.k = [100., 0., 9.5, 0., 100., 5.5, 0., 0., 1.]
        info.p = [100., 0., 9.5, 0., 0., 100., 5.5, 0., 0., 0., 1., 0.]
    return msg


def test_detection_to_icp_transport_tf_required_and_source_identity():
    rclpy.init()
    ex = SingleThreadedExecutor()
    source = rclpy.create_node('synthetic_perception_source')
    detector = DetectionNode(detector=SyntheticDetector(), parameter_overrides=parameters(width=20, height=12))
    v, u = np.mgrid[:12, :20]
    model = np.column_stack(((u.ravel()-9.5)*.005, (v.ravel()-5.5)*.005, np.zeros(240)))
    normals = np.tile([0., 0., -1.], (len(model), 1))
    icp = IcpNode(model_data=(model, normals, 'synthetic-centered-plane'),
                  parameter_overrides=parameters(width=20, height=12,
                                                 reference_frame='synthetic_plane'))
    for node in (source, detector, icp):
        ex.add_node(node)
    frames, poses = [], []
    subscriptions = [
        source.create_subscription(DetectionFrame, '/task/detection_frames', frames.append, qos_profile_sensor_data),
        source.create_subscription(ObjectDetectionArray, '/task/object_detections', poses.append, qos_profile_sensor_data)]
    pub = source.create_publisher(Observation, '/task/observation', qos_profile_sensor_data)
    sent = {}
    sequence = 0
    def spin_until(predicate, timeout=6):
        nonlocal sequence
        deadline = time.monotonic()+timeout
        next_publish = 0.
        while time.monotonic() < deadline and not predicate():
            if time.monotonic() >= next_publish:
                msg = observation(source, sequence)
                sent[sequence] = msg
                pub.publish(msg)
                sequence += 1
                next_publish = time.monotonic()+.15
            ex.spin_once(timeout_sec=.02)
        assert predicate(), (detector.status, icp.status)
    try:
        spin_until(lambda: icp.rejected > 0 and bool(frames))
        assert not poses  # Missing TF must never become an identity base pose.
        broadcaster = StaticTransformBroadcaster(source)
        transforms = []
        for parent, child, x in [('base_link', 'synthetic_plane', .2),
                                  ('synthetic_plane', 'synthetic_optical', 0.)]:
            transform = TransformStamped()
            transform.header.stamp = source.get_clock().now().to_msg()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.translation.x = x
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        broadcaster.sendTransform(transforms)
        spin_until(lambda: len(poses) >= 2)
        result = poses[-1]
        validate_objects(result, 'base_link')
        assert result.header.stamp == sent[result.sequence].header.stamp
        assert len(result.detections) == 1
        obj = result.detections[0]
        assert not obj.full_orientation
        assert obj.fit_metrics_valid and not obj.covariance_estimated
        assert obj.trimmed_rmse_m < .015 and obj.target_coverage_10mm >= .5
        assert obj.track_id == poses[-2].detections[0].track_id
        np.testing.assert_allclose([obj.pose.pose.position.x, obj.pose.pose.position.y,
                                    obj.pose.pose.position.z], [.2, 0., .5], atol=.012)
        f = next(f for f in frames if f.sequence == result.sequence)
        assert f.depth.header == sent[result.sequence].depth.header
        assert bytes(f.depth.data) == bytes(sent[result.sequence].depth.data)
        before = detector.rejected
        detector.enqueue(sent[result.sequence])
        assert detector.rejected == before+1
        stale = observation(source, 999)
        stale.header.stamp.sec -= 5
        detector.enqueue(stale)
        assert detector.rejected == before+2
        mismatched = deepcopy(f)
        mismatched.depth.header.stamp.sec -= 1
        with pytest.raises(ValueError, match='acquisition mismatch'):
            icp.estimate((mismatched, np.eye(4), np.eye(4), icp.epoch))
        empty = deepcopy(f)
        empty.depth.data = np.full((12, 20), np.nan, dtype='<f4').tobytes()
        assert not icp.estimate((empty, np.eye(4), np.eye(4), icp.epoch)).detections
    finally:
        ex.shutdown()
        for node in (detector, icp, source):
            node.destroy_node()
        rclpy.shutdown()


def test_finished_worker_result_cannot_publish_after_expiry_or_clock_epoch_change():
    rclpy.init()
    node = PipelineNode('expiry_test', {})
    class Publisher:
        def publish(self, msg):
            pytest.fail('invalid worker result published')
    node.publisher = Publisher()
    node.start_worker(lambda msg: msg)
    try:
        stale = Observation()
        stale.header.stamp = node.get_clock().now().to_msg()
        stale.header.stamp.sec -= 5
        completed = Future()
        completed.set_result((node.epoch, stale))
        node.worker.future = completed
        node._poll()
        assert node.rejected == 1
        fresh = Observation()
        fresh.header.stamp = node.get_clock().now().to_msg()
        completed = Future()
        completed.set_result((node.epoch-1, fresh))
        node.worker.future = completed
        node._poll()
        assert node.rejected == 2 and node.accepted == 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_shared_detection_publishes_without_tf_and_mode_change_discards_work():
    from deux_task_perception.merged_perception_node import PerceptionNode
    from deux_task_msgs.msg import DetectionArray, PerceptionRequest
    from builtin_interfaces.msg import Time as Stamp
    rclpy.init()
    ex = SingleThreadedExecutor()
    source = rclpy.create_node('shared_source')
    class Detector(SyntheticDetector):
        def __call__(self, rgb):
            return super().__call__(rgb)+[(0,'label',.95,[5.,2.,14.,9.])]
    node = PerceptionNode(detector=Detector(), model_data=(np.zeros((40,3)),np.tile([0.,0.,1.],(40,1)),'test'),
        parameter_overrides=parameters(width=20,height=12,reference_frame='plane',barcode_classes=['label']))
    ex.add_node(source); ex.add_node(node)
    found=[]
    sub=source.create_subscription(DetectionArray,'/task/detections',found.append,qos_profile_sensor_data)
    try:
        deadline=time.monotonic()+3
        while not node.detection_pub.get_subscription_count() and time.monotonic()<deadline: ex.spin_once(timeout_sec=.02)
        msg=observation(source,0)
        node.receive_images(msg.rgb,msg.depth,msg.rgb_info,msg.depth_info)
        while not found and time.monotonic()<deadline: ex.spin_once(timeout_sec=.02)
        assert {b.class_name for b in found[-1].boxes} == {'s1','label'}
        assert found[-1].header == msg.header and found[-1].sequence == 0
        old=node.token()
        req=PerceptionRequest(request_id='new',target_id='bag',mode='barcode')
        req.header.stamp=source.get_clock().now().to_msg()
        req.expires_at=Stamp(sec=req.header.stamp.sec+5,nanosec=req.header.stamp.nanosec)
        node.set_request(req)
        assert not node.admitted(old,msg.header)
        assert node.admitted(node.token(),msg.header)
        node.request.expires_at=Stamp(sec=1)
        assert not node.admitted(node.token(),msg.header)
        # TF errors from independent ICP must not suppress common detections.
        deadline=time.monotonic()+2
        while node.rejected == 0 and time.monotonic()<deadline: ex.spin_once(timeout_sec=.02)
        assert node.rejected > 0
    finally:
        ex.shutdown();node.destroy_node();source.destroy_node();rclpy.shutdown()


def test_barcode_surface_carries_request_acquisition_and_rejects_ambiguous_tracks():
    from deux_task_perception.merged_perception_node import PerceptionNode
    from deux_task_msgs.msg import PerceptionRequest, ObjectDetection
    rclpy.init()
    node=PerceptionNode(detector=SyntheticDetector(),model_data=(np.zeros((40,3)),np.tile([0.,0.,1.],(40,1)),'test'),
        parameter_overrides=parameters(width=20,height=12,reference_frame='plane',barcode_classes=['label']))
    try:
        msg=observation(node,0)
        req=PerceptionRequest(request_id='scan-1',target_id='bag',mode='barcode')
        _,_,frame=node.detect((node.token(),req,(msg.rgb,msg.depth,msg.rgb_info,msg.depth_info),42))
        from deux_task_msgs.msg import BoundingBox
        frame.boxes.append(BoundingBox(class_name='label',confidence=.9,xyxy=[1.,1.,18.,10.]))
        objects=ObjectDetectionArray(header=deepcopy(msg.header))
        obj=ObjectDetection(track_id='bag');obj.pose.pose.position.z=.5;obj.pose.pose.orientation.w=1.
        objects.detections=[obj]
        node.transforms=lambda _: (np.eye(4),np.eye(4))
        _,result=node.label((node.token(),req,frame,objects))
        assert result.request_id == 'scan-1' and result.target_id == 'bag'
        assert result.sequence == 42 and result.header.stamp == msg.header.stamp
        assert result.orientation_valid and result.header.frame_id == 'base_link'
        other=deepcopy(obj);other.track_id='other';objects.detections.append(other)
        with pytest.raises(ValueError,match='unique'): node.label((node.token(),req,frame,objects))
    finally:
        node.destroy_node();rclpy.shutdown()


def test_lightweight_state_preserves_object_acquisition_without_images():
    from deux_task_perception.state_observation_node import StateObservationNode
    from deux_task_msgs.msg import StateObservation, ObjectDetection
    rclpy.init();node=StateObservationNode();source=rclpy.create_node('state_check')
    ex=SingleThreadedExecutor();ex.add_node(node);ex.add_node(source);found=[]
    sub=source.create_subscription(StateObservation,'/task/state_observation',found.append,qos_profile_sensor_data)
    try:
        msg=ObjectDetectionArray();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='base_link'
        msg.source='synthetic';msg.model_id='fixture';msg.sequence=17
        d=ObjectDetection(track_id='bag',confidence=.9);d.pose.pose.orientation.w=1.
        msg.detections=[d];node.receive_objects(msg)
        deadline=time.monotonic()+3
        while not found and time.monotonic()<deadline: ex.spin_once(timeout_sec=.02)
        assert found and found[-1].objects_valid and not found[-1].joints_valid
        assert found[-1].objects.header == msg.header and found[-1].objects.sequence == 17
        assert not hasattr(found[-1],'rgb')
    finally:
        ex.shutdown();node.destroy_node();source.destroy_node();rclpy.shutdown()
