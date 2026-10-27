"""Tracked RGB-D perception with independent detection and ICP workers.

Only the executor handles requests, queues, TF lookup and ROS publication.
Workers own their tracker/filter state. Frames passed between workers are
read-only references to the same acquisition, never a new depth sample.
"""
from rclpy.time import Time
from deux_task_msgs.msg import ObjectDetectionArray
from .tracked_detection_node import TrackedDetectionNode
from .icp_estimator import IcpEstimator, ICP_DEFAULTS
from .icp_node import matrix
from .perception_core import LatestWorker
from .pipeline_node import run, LATEST_QOS
from .core import stamp_ns


class TrackedPerceptionNode(TrackedDetectionNode):
    def __init__(self, detector=None, model_data=None, **kwargs):
        defaults = dict(ICP_DEFAULTS)
        # Tracking expiry and ICP/Kalman expiry have different meanings.
        defaults.pop('track_max_age_s')
        defaults.update(input_topic='/task/observation', target_classes=['bread_bag'],
                        roi_only_depth=True, require_instance_mask=True,
                        icp_track_max_age_s=.6)
        super().__init__(detector, node_name='perception_node',
                         extra_defaults=defaults, icp_transport=False, **kwargs)
        config = dict(self.cfg, track_max_age_s=self.cfg['icp_track_max_age_s'])
        self.estimator = IcpEstimator(config, model_data)
        self.generation = 0
        self.icp_accepted = self.icp_rejected = 0
        self.icp_publisher = self.create_publisher(
            ObjectDetectionArray, self.cfg['output_topic'], LATEST_QOS)
        self.icp_completion = self.create_guard_condition(self.finish_icp)
        self.icp_worker = LatestWorker(self.fit, self.icp_completion.trigger)

    def set_request(self, msg):
        old = self.request
        super().set_request(msg)
        if self.request is not old:
            self.generation += 1
            self.icp_worker.pending = None

    def now(self):
        epoch = self.epoch
        now = super().now()
        if epoch != self.epoch and hasattr(self, 'icp_worker'):
            self.generation += 1
            self.icp_worker.pending = None
        return now

    def submit_icp(self, frame):
        try:
            when = Time.from_msg(frame.header.stamp, clock_type=self.get_clock().clock_type)
            reference = self.cfg['reference_frame']
            rc = matrix(self.tf.lookup_transform(reference, frame.header.frame_id, when))
            br = matrix(self.tf.lookup_transform(self.cfg['base_frame'], reference, when))
            token = (self.epoch, self.generation)
            self.icp_worker.submit((token, frame, rc, br))
        except Exception as error:
            self.icp_rejected += 1
            self.reject(f'ICP input: {error}')

    def fit(self, job):
        token, frame, rc, br = job
        # A changed lease/epoch resets the filter INSIDE its sole owner worker.
        # Even a superseded in-flight fit cannot contaminate the new generation.
        return token, self.estimator.estimate((frame, rc, br, token))

    def finish_icp(self):
        now = self.now()
        req = self.request
        if req is None or now >= self.expiry(req) or req.mode != 'locate':
            self.icp_worker.pending = None
        done = self.icp_worker.poll()
        if done is None:
            return
        try:
            token, result = done.result()
            if (token != (self.epoch, self.generation) or req is None
                    or req.mode != 'locate' or now >= self.expiry(req)
                    or result.request_id != req.request_id
                    or not self.fresh(result.header, now)
                    or stamp_ns(result.header) < stamp_ns(req.header)
                    or any(d.track_id != req.target_id for d in result.detections)):
                raise ValueError('superseded or expired ICP result')
            self.icp_publisher.publish(result)
            self.icp_accepted += 1
        except Exception as error:
            self.icp_rejected += 1
            self.reject(f'ICP result: {error}')

    def destroy_node(self):
        if hasattr(self, 'icp_worker'):
            self.icp_worker.close()
        return super().destroy_node()


def main(args=None):
    run(TrackedPerceptionNode, args)
