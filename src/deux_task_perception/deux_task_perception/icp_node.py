"""Reference-plane yaw ICP; only explicitly calibrated TF produces base poses."""
from copy import deepcopy
import math
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener, TransformException
from deux_task_msgs.msg import DetectionFrame, ObjectDetection, ObjectDetectionArray
from .core import normalize_depth, stamp_ns
from .perception_core import load_model, roi_points, transform_matrix, Tracker
from .pipeline_node import PipelineNode, run
from .yaw_icp import fit_icp


def matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return transform_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


class IcpNode(PipelineNode):
    def __init__(self, model_data=None, **kwargs):
        super().__init__('icp_node', {
            'input_topic': '/task/detection_frames', 'output_topic': '/task/object_detections',
            'model_path': '', 'model_origin': '', 'reference_frame': '', 'base_frame': 'base_link',
            'min_height_m': .01, 'max_distance_m': 1.5, 'max_rmse_m': .015,
            'min_coverage': .5, 'track_gate_m': .08, 'track_max_age_s': .6,
            'position_std_m': .02, 'yaw_std_rad': .2, 'unobserved_std_rad': math.pi,
            'width': 640, 'height': 480, 'sync_slop_s': .02,
            'target_classes': ['s1', 's1_1']}, **kwargs)
        self.positive('max_distance_m', 'max_rmse_m', 'track_gate_m', 'track_max_age_s',
                      'position_std_m', 'yaw_std_rad', 'unobserved_std_rad',
                      'width', 'height', 'sync_slop_s')
        if (not self.cfg['reference_frame'] or not self.cfg['base_frame'] or
                not math.isfinite(self.cfg['min_height_m']) or
                not 0 <= self.cfg['min_coverage'] <= 1):
            raise ValueError('explicit reference/base frames and finite height/coverage required')
        self.model, self.normals, self.model_id = model_data if model_data is not None else load_model(
            self.cfg['model_path'], self.cfg['model_origin'])
        self.tracker = Tracker(self.cfg['track_gate_m'], self.cfg['track_max_age_s'])
        self.track_epoch = None
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(ObjectDetectionArray, self.cfg['output_topic'], qos_profile_sensor_data)
        self.start_worker(self.estimate)
        self.subscription = self.create_subscription(DetectionFrame, self.cfg['input_topic'],
                                                      self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        try:
            self.now()
            when = Time.from_msg(msg.header.stamp, clock_type=self.get_clock().clock_type)
            # Identity is legitimate only when the explicitly selected frames coincide.
            reference = self.cfg['reference_frame']
            rc = matrix(self.tf.lookup_transform(reference, msg.header.frame_id, when))
            br = matrix(self.tf.lookup_transform(self.cfg['base_frame'], reference, when))
            self.enqueue(msg, (msg, rc, br, self.epoch))
        except (TransformException, ValueError) as error:
            self.reject(f'calibrated TF unavailable: {error}')

    def estimate(self, payload):
        frame, rc, br, epoch = payload
        if epoch != self.track_epoch:
            self.tracker.tracks.clear()
            self.track_epoch = epoch
        depth, _ = normalize_depth(frame.depth)
        info = frame.depth_info
        headers = [frame.header, frame.depth.header, info.header]
        times = [stamp_ns(h) for h in headers]
        if (len({h.frame_id for h in headers}) != 1 or not frame.header.frame_id or
                min(times) <= 0 or max(times)-min(times) > self.cfg['sync_slop_s']*1e9 or
                depth.shape != (self.cfg['height'], self.cfg['width']) or
                (info.height, info.width) != depth.shape or
                info.binning_x not in (0, 1) or info.binning_y not in (0, 1) or
                info.roi.x_offset or info.roi.y_offset or
                info.roi.width not in (0, info.width) or info.roi.height not in (0, info.height)):
            raise ValueError('depth/calibration acquisition mismatch')
        out = ObjectDetectionArray()
        out.header = deepcopy(frame.header)
        out.header.frame_id = self.cfg['base_frame']
        out.sequence, out.source, out.model_id = frame.sequence, 'yaw_icp/' + frame.detector_id, self.model_id
        poses, scores, metrics = [], [], []
        for box in frame.boxes:
            if box.class_name not in self.cfg['target_classes']:
                continue
            if not math.isfinite(box.confidence) or not 0 <= box.confidence <= 1:
                raise ValueError('invalid detection confidence')
            points = roi_points(depth, info.p, box.xyxy, rc,
                                self.cfg['min_height_m'], self.cfg['max_distance_m'])
            fit = fit_icp(points, self.model, self.normals, rc[:3, 3])
            if (fit['status'] != 'fitted' or
                    not np.isfinite([fit['trimmed_rmse_m'], fit['target_coverage_10mm']]).all() or
                    fit['trimmed_rmse_m'] > self.cfg['max_rmse_m'] or
                    fit['target_coverage_10mm'] < self.cfg['min_coverage']):
                continue
            pose = br @ np.array(fit['T_reference_model'])
            if not np.isfinite(pose).all():
                continue
            poses.append(pose)
            scores.append(box.confidence)
            metrics.append((fit['trimmed_rmse_m'], fit['target_coverage_10mm']))
        ids = self.tracker.update([p[:3, 3] for p in poses], stamp_ns(frame.header)*1e-9)
        # Configured uncertainty assumptions, not covariance estimated from ICP residuals.
        cov = np.diag([self.cfg['position_std_m']**2]*3 +
                      [self.cfg['unobserved_std_rad']**2]*2 + [self.cfg['yaw_std_rad']**2])
        axes = np.zeros((6, 6))
        axes[:3, :3] = axes[3:, 3:] = br[:3, :3]
        cov = axes @ cov @ axes.T
        for identity, pose, score, (rmse, coverage) in zip(ids, poses, scores, metrics):
            detection = ObjectDetection(track_id=identity, confidence=score, full_orientation=False)
            detection.fit_metrics_valid = True
            detection.trimmed_rmse_m, detection.target_coverage_10mm = rmse, coverage
            detection.covariance_estimated = False
            p, q = detection.pose.pose.position, detection.pose.pose.orientation
            p.x, p.y, p.z = map(float, pose[:3, 3])
            q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(pose[:3, :3]).as_quat())
            detection.pose.covariance = cov.ravel().tolist()
            out.detections.append(detection)
        return out


def main(args=None):
    run(IcpNode, args)
