"""One selected native RGB-D bundle -> GPU YOLO -> ROI depth -> yaw ICP."""
from copy import deepcopy
from types import SimpleNamespace
import time
import numpy as np
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from deux_task_msgs.msg import DetectionFrame, BoundingBox
from .core import normalize_rgb, validate_bundle, stamp_ns
from .icp_node import IcpNode, matrix
from .pipeline_node import run
from .yolo_backend import YoloDetector


class PerceptionNode(IcpNode):
    def __init__(self, detector=None, model_data=None, **kwargs):
        super().__init__(model_data=model_data, input_enabled=False, node_name='perception_node',
                         extra_defaults={
                             'weights': '', 'device': 'cpu', 'confidence': .25,
                             'rgb_topic': '/zed/zed_node/rgb/color/rect/image',
                             'depth_topic': '/zed/zed_node/depth/depth_registered',
                             'rgb_info_topic': '/zed/zed_node/rgb/color/rect/camera_info',
                             'depth_info_topic': '/zed/zed_node/depth/camera_info',
                             'sync_queue_size': 3, 'max_image_age_s': .5,
                             'roi_only_depth': True}, **kwargs)
        self.positive('confidence', 'max_image_age_s', 'sync_queue_size')
        if self.cfg['confidence'] > 1:
            raise ValueError('confidence must be <= 1')
        self.detector = detector if detector is not None else YoloDetector(
            self.cfg['weights'], self.cfg['device'], self.cfg['confidence'])
        self.sequence = 0
        self.last_timings = {}
        # Synchronizer needs a small bounded history; worker retains only latest pending bundle.
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=3,
                         reliability=ReliabilityPolicy.BEST_EFFORT)
        self.filters = [message_filters.Subscriber(self, typ, self.cfg[key], qos_profile=qos)
                        for typ, key in ((Image, 'rgb_topic'), (Image, 'depth_topic'),
                                         (CameraInfo, 'rgb_info_topic'), (CameraInfo, 'depth_info_topic'))]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.filters, self.cfg['sync_queue_size'], self.cfg['sync_slop_s'])
        self.sync.registerCallback(self.receive_images)

    def receive_images(self, rgb, depth, rgb_info, depth_info):
        # No normalization, image copy, inference, or TF lookup on discarded pending frames.
        self.now()
        payload = (rgb, depth, rgb_info, depth_info, self.sequence, self.epoch)
        self.sequence += 1
        self.enqueue(SimpleNamespace(header=rgb.header), payload)

    def estimate(self, payload):
        rgb, depth, rgb_info, depth_info, sequence, epoch = payload
        began = time.monotonic()
        now = self.get_clock().now().nanoseconds
        times = [stamp_ns(m.header) for m in (rgb, depth, rgb_info, depth_info)]
        if (min(times) <= 0 or max(times) > now or
                now-min(times) > self.cfg['max_image_age_s']*1e9):
            raise ValueError('stale or future synchronized RGB-D')
        validate_bundle(rgb, depth, rgb_info, depth_info,
                        self.cfg['width'], self.cfg['height'], int(self.cfg['sync_slop_s']*1e9))
        when = Time.from_msg(rgb.header.stamp, clock_type=self.get_clock().clock_type)
        rc = matrix(self.tf.lookup_transform(self.cfg['reference_frame'], rgb.header.frame_id, when))
        br = matrix(self.tf.lookup_transform(self.cfg['base_frame'], self.cfg['reference_frame'], when))
        frame = DetectionFrame(header=deepcopy(rgb.header), sequence=sequence,
                               detector_id=self.detector.identity)
        # Local references only. This DetectionFrame is never serialized/published.
        frame.depth, frame.depth_info = depth, depth_info
        yolo_start = time.monotonic()
        for cls, name, confidence, box in self.detector(normalize_rgb(rgb)):
            if name not in self.cfg['target_classes']:
                continue
            if (len(box) != 4 or not np.isfinite([confidence, *box]).all() or
                    not self.cfg['confidence'] <= confidence <= 1 or
                    box[2] <= box[0] or box[3] <= box[1]):
                raise ValueError('invalid detector result')
            frame.boxes.append(BoundingBox(class_id=cls, class_name=name,
                                          confidence=confidence, xyxy=list(map(float, box))))
        icp_start = time.monotonic()
        result = super().estimate((frame, rc, br, epoch))
        self.last_timings = {'yolo_ms': (icp_start-yolo_start)*1000,
                             'icp_ms': (time.monotonic()-icp_start)*1000,
                             'total_compute_ms': (time.monotonic()-began)*1000}
        return result


def main(args=None):
    run(PerceptionNode, args)
