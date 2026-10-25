"""Shared YOLO with independent latest-only ICP and label workers.

Images remain local; detection metadata is published before expensive ICP.
Requests are bounded leases; old in-flight results cannot escape a mode change.
"""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import message_filters
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from deux_task_msgs.msg import DetectionFrame, BoundingBox, DetectionArray, PerceptionRequest, BarcodeTarget
from .core import normalize_rgb, normalize_depth, validate_bundle, stamp_ns
from .icp_node import IcpNode, matrix
from .perception_core import LatestWorker
from .pipeline_node import run
from .yolo_backend import YoloDetector
from .barcode_geometry import surface, associated_bag


class PerceptionNode(IcpNode):
    def __init__(self, detector=None, model_data=None, **kwargs):
        super().__init__(model_data=model_data, input_enabled=False, node_name='perception_node',
                         extra_defaults={
                             'weights': '', 'device': 'cpu', 'confidence': .25,
                             'bag_classes': ['s1', 's1_1'], 'barcode_classes': ['__configure_barcode_class__'],
                             'rgb_topic': '/zed/zed_node/rgb/color/rect/image',
                             'depth_topic': '/zed/zed_node/depth/depth_registered',
                             'rgb_info_topic': '/zed/zed_node/rgb/color/rect/camera_info',
                             'depth_info_topic': '/zed/zed_node/depth/camera_info',
                             'sync_queue_size': 3, 'max_image_age_s': .5,
                             'roi_only_depth': True}, **kwargs)
        self.positive('confidence', 'max_image_age_s', 'sync_queue_size')
        if self.cfg['confidence'] > 1 or set(self.cfg['bag_classes']) & set(self.cfg['barcode_classes']):
            raise ValueError('invalid confidence or overlapping bag/barcode classes')
        self.cfg['target_classes'] = self.cfg['bag_classes']
        self.detector = detector if detector is not None else YoloDetector(
            self.cfg['weights'], self.cfg['device'], self.cfg['confidence'])
        self.worker.close()
        self.worker = LatestWorker(self.detect)
        self.icp_worker = LatestWorker(self.fit)
        self.label_worker = LatestWorker(self.label)
        self.request = None
        self.generation = 0
        self.last_request_stamp = 0
        self.sequence = 0
        self.latest_objects = None
        self.detection_pub = self.create_publisher(DetectionArray, '/task/detections', qos_profile_sensor_data)
        self.label_pub = self.create_publisher(BarcodeTarget, '/task/barcode_target', qos_profile_sensor_data)
        self.request_sub = self.create_subscription(PerceptionRequest, '/task/perception_request', self.set_request,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.filters = [message_filters.Subscriber(self, typ, self.cfg[key], qos_profile=qos)
                        for typ, key in ((Image, 'rgb_topic'), (Image, 'depth_topic'),
                                         (CameraInfo, 'rgb_info_topic'), (CameraInfo, 'depth_info_topic'))]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.filters, self.cfg['sync_queue_size'], self.cfg['sync_slop_s'])
        self.sync.registerCallback(self.receive_images)

    def set_request(self, req):
        now = self.now()
        if (not req.request_id or req.mode not in ('locate', 'track', 'barcode', 'idle') or
                not 0 < stamp_ns(req.header) <= now < self.expiry(req) or
                self.expiry(req)-now > 120e9 or
                (req.mode == 'barcode' and not req.target_id)):
            self.reject('invalid perception lease'); return
        if stamp_ns(req.header) <= self.last_request_stamp:
            self.reject('out-of-order perception request'); return
        self.last_request_stamp = stamp_ns(req.header)
        self.request = deepcopy(req)
        self.generation += 1
        for worker in (self.worker, self.icp_worker, self.label_worker): worker.pending = None

    def now(self):
        old_epoch = self.epoch
        now = super().now()
        if hasattr(self, 'request'):
            reset = self.epoch != old_epoch
            if reset or (self.request is not None and now >= self.expiry(self.request)):
                # Expired control returns to passive localization, never barcode motion.
                self.request = None
                self.generation += 1
                if reset:
                    self.last_request_stamp = 0
                    self.latest_objects = None
                for worker in (self.worker, self.icp_worker, self.label_worker): worker.pending = None
        return now

    @staticmethod
    def expiry(req):
        return req.expires_at.sec*10**9 + req.expires_at.nanosec

    def token(self): return (self.epoch, self.generation)

    def admitted(self, token, header):
        now = self.now()
        return (token == self.token() and self.fresh(header, now) and
                (self.request is None or now < self.expiry(self.request)))

    def receive_images(self, rgb, depth, rgb_info, depth_info):
        now = self.now()
        if not self.fresh(rgb.header, now) or (self.last_input is not None and stamp_ns(rgb.header) <= self.last_input): return
        if self.request and (now >= self.expiry(self.request) or self.request.mode == 'idle'): return
        self.last_input = stamp_ns(rgb.header)
        self.worker.submit((self.token(), deepcopy(self.request), (rgb, depth, rgb_info, depth_info), self.sequence))
        self.sequence += 1

    def detect(self, job):
        token, req, bundle, sequence = job
        rgb, depth, ri, di = bundle
        validate_bundle(rgb, depth, ri, di, self.cfg['width'], self.cfg['height'], int(self.cfg['sync_slop_s']*1e9))
        now = self.get_clock().now().nanoseconds
        if any(not 0 < stamp_ns(m.header) <= now or now-stamp_ns(m.header) > self.cfg['max_image_age_s']*1e9 for m in bundle):
            raise ValueError('expired RGB-D bundle')
        frame = DetectionFrame(header=deepcopy(rgb.header), sequence=sequence, detector_id=self.detector.identity,
                               depth=depth, depth_info=di)
        for cls, name, confidence, box in self.detector(normalize_rgb(rgb)):
            if name not in self.cfg['bag_classes'] + self.cfg['barcode_classes']: continue
            if len(box) != 4 or not np.isfinite([confidence, *box]).all() or not self.cfg['confidence'] <= confidence <= 1 or box[2] <= box[0] or box[3] <= box[1]:
                raise ValueError('invalid detector output')
            frame.boxes.append(BoundingBox(class_id=cls, class_name=name, confidence=float(confidence), xyxy=list(map(float,box))))
        return token, req, frame

    def transforms(self, frame):
        when = Time.from_msg(frame.header.stamp, clock_type=self.get_clock().clock_type)
        rc = matrix(self.tf.lookup_transform(self.cfg['reference_frame'], frame.header.frame_id, when))
        br = matrix(self.tf.lookup_transform(self.cfg['base_frame'], self.cfg['reference_frame'], when))
        return rc, br

    def fit(self, job):
        token, req, frame = job
        rc, br = self.transforms(frame)
        result = IcpNode.estimate(self, (frame, rc, br, token[0]))
        result.request_id = req.request_id if req else ""
        return token, result

    def label(self, job):
        token, req, frame, objects = job
        if objects is None or abs(stamp_ns(frame.header)-stamp_ns(objects.header)) > self.cfg['track_max_age_s']*1e9:
            raise ValueError('fresh bag track required for label association')
        targets = [d for d in objects.detections if d.track_id == req.target_id]
        if len(targets) != 1: raise ValueError('target track missing')
        rc, br = self.transforms(frame); bc = br@rc
        t = targets[0].pose.pose.position
        camera_target = (np.linalg.inv(bc) @ [t.x,t.y,t.z,1.])[:3]
        bags = [list(b.xyxy) for b in frame.boxes if b.class_name in self.cfg['bag_classes']]
        depth, _ = normalize_depth(frame.depth)
        candidates = []
        for box in frame.boxes:
            if box.class_name not in self.cfg['barcode_classes']: continue
            try:
                associated_bag(list(box.xyxy), bags, camera_target, frame.depth_info.p)
                pose, rmse, fraction = surface(depth, frame.depth_info.p, box.xyxy, self.cfg['max_distance_m'])
                if np.linalg.norm(pose[:3,3]-camera_target) > .2: continue
                # Reject another tracked bag projected into the same enclosing box.
                for other in objects.detections:
                    if other.track_id == req.target_id: continue
                    p = other.pose.pose.position
                    try: associated_bag(list(box.xyxy), bags, (np.linalg.inv(bc)@[p.x,p.y,p.z,1.])[:3], frame.depth_info.p)
                    except ValueError: continue
                    raise ValueError('multiple tracks inside bag bbox')
                out = BarcodeTarget(header=deepcopy(frame.header), sequence=frame.sequence,
                    request_id=req.request_id, target_id=req.target_id, plane_rmse_m=rmse,
                    valid_fraction=fraction, orientation_valid=True)
                out.header.frame_id = self.cfg['base_frame']; pose = bc@pose
                p,q = out.pose.position,out.pose.orientation
                p.x,p.y,p.z = map(float,pose[:3,3]); q.x,q.y,q.z,q.w = map(float,Rotation.from_matrix(pose[:3,:3]).as_quat())
                candidates.append(out)
            except ValueError: continue
        if len(candidates) != 1: raise ValueError('no unique valid barcode surface')
        return token,candidates[0]

    def _poll(self):
        self.now()
        if not hasattr(self, 'label_worker'): return
        for worker, kind in ((self.worker,'detect'),(self.icp_worker,'icp'),(self.label_worker,'label')):
            done = worker.poll()
            if done is None: continue
            try:
                value = done.result(); token = value[0]; result = value[-1]
                if not self.admitted(token,result.header): raise ValueError('superseded/expired perception result')
                if kind == 'detect':
                    _,req,frame = value
                    self.detection_pub.publish(DetectionArray(header=frame.header,sequence=frame.sequence,
                        request_id=req.request_id if req else '',detector_id=frame.detector_id, boxes=frame.boxes))
                    # Keep independent bag monitoring in barcode mode; never synthesize bag pose from robot FK.
                    self.icp_worker.submit(value)
                    if req and req.mode == 'barcode': self.label_worker.submit((*value,deepcopy(self.latest_objects)))
                elif kind == 'icp':
                    self.latest_objects = result; self.publisher.publish(result)
                else: self.label_pub.publish(result)
                self.accepted += 1
            except Exception as error: self.reject(str(error))

    def destroy_node(self):
        for name in ('icp_worker','label_worker'):
            if hasattr(self,name): getattr(self,name).close()
        return super().destroy_node()


def main(args=None): run(PerceptionNode,args)
