"""Tracked RGB-D perception with independent detection and ICP workers.

Only the executor handles requests, queues, TF lookup and ROS publication.
Workers own their tracker/filter state. Frames passed between workers are
read-only references to the same acquisition, never a new depth sample.
"""
import time
import json
from std_msgs.msg import String
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
                        roi_only_depth=True, require_instance_mask=False,
                        icp_track_max_age_s=.6)
        super().__init__(detector, node_name='perception_node',
                         extra_defaults=defaults, icp_transport=False, **kwargs)
        config = dict(self.cfg, track_max_age_s=self.cfg['icp_track_max_age_s'])
        self.estimator = IcpEstimator(config, model_data)
        self.generation = 0
        self.icp_accepted = self.icp_rejected = 0
        self.icp_publisher = self.create_publisher(
            ObjectDetectionArray, self.cfg['output_topic'], LATEST_QOS)
        self.timing_publisher = self.create_publisher(String, '/task/icp_timing', 10)
        self.icp_completion = self.create_guard_condition(self.finish_icp)
        self.icp_worker = LatestWorker(self.fit, self.icp_completion.trigger)

    def _recover_completed(self):
        super()._recover_completed()
        worker = getattr(self, 'icp_worker', None)
        if worker is not None and worker.future is not None and worker.future.done():
            self.finish_icp()

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
        submitted = time.perf_counter()
        submitted_ros = self.get_clock().now().nanoseconds
        try:
            when = Time.from_msg(frame.header.stamp, clock_type=self.get_clock().clock_type)
            reference = self.cfg['reference_frame']
            rc = matrix(self.tf.lookup_transform(reference, frame.header.frame_id, when))
            br = matrix(self.tf.lookup_transform(self.cfg['base_frame'], reference, when))
            token = (self.epoch, self.generation)
            self.icp_worker.submit((token, frame, rc, br, submitted, submitted_ros, time.perf_counter()))
        except Exception as error:
            self.icp_rejected += 1
            self.reject(f'ICP input: {error}')

    def fit(self, job):
        token, frame, rc, br, submitted, submitted_ros, queued = job
        start = time.perf_counter()
        result, error = None, None
        self.estimator.last_timing = {}
        try:
            result = self.estimator.estimate((frame, rc, br, token))
        except Exception as exc:
            error = str(exc)
        end = time.perf_counter()
        timing = dict(sequence=int(frame.sequence),request_id=frame.request_id,
            acquisition_ns=stamp_ns(frame.header),input_to_submit_ms=(submitted_ros-stamp_ns(frame.header))/1e6,
            tf_submit_ms=(queued-submitted)*1000,queue_wait_ms=(start-queued)*1000,
            worker_ms=(end-start)*1000,worker_finished=end,submitted=submitted,
            estimator=dict(self.estimator.last_timing),error=error)
        return token, result, timing

    def finish_icp(self):
        now = self.now()
        req = self.request
        if req is None or now >= self.expiry(req) or req.mode != 'locate':
            self.icp_worker.pending = None
        done = self.icp_worker.poll()
        if done is None:
            return
        timing = None
        try:
            token, result, timing = done.result()
            timing["completion_wait_ms"] = (time.perf_counter()-timing.pop("worker_finished"))*1000
            if timing["error"] is not None:
                raise ValueError(timing["error"])
            if (token != (self.epoch, self.generation) or req is None
                    or req.mode != 'locate' or now >= self.expiry(req)
                    or result.request_id != req.request_id
                    or not self.fresh(result.header, now)
                    or stamp_ns(result.header) < stamp_ns(req.header)
                    or any(d.track_id != req.target_id for d in result.detections)):
                raise ValueError('superseded or expired ICP result')
            publish_start = time.perf_counter()
            self.icp_publisher.publish(result)
            timing["publish_ms"] = (time.perf_counter()-publish_start)*1000
            timing["accepted"] = True
            self.icp_accepted += 1
        except Exception as error:
            self.icp_rejected += 1
            self.reject(f'ICP result: {error}')
            if timing is not None:
                timing.update(accepted=False,error=str(error))
        finally:
            if timing is not None:
                timing['submit_to_finish_ms'] = (time.perf_counter()-timing.pop('submitted'))*1000
                timing['acquisition_to_finish_ms'] = (self.get_clock().now().nanoseconds-timing['acquisition_ns'])/1e6
                timing['queue_replaced_total'] = self.icp_worker.dropped
                self.timing_publisher.publish(String(data=json.dumps(timing)))
                # Persist rejection details without flooding the executor log.
                if not timing.get('accepted', False):
                    clock = time.monotonic()
                    if clock - getattr(self, '_last_icp_rejection_log', 0.) >= 1.:
                        self._last_icp_rejection_log = clock
                        self.get_logger().warning("ICP rejected: " + json.dumps(timing))


    def destroy_node(self):
        if hasattr(self, 'icp_worker'):
            self.icp_worker.close()
        return super().destroy_node()


def main(args=None):
    run(TrackedPerceptionNode, args)
