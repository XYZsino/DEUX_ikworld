"""Reference-plane yaw ICP; only explicitly calibrated TF produces base poses."""
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
from deux_task_msgs.msg import DetectionFrame, ObjectDetectionArray
from .perception_core import transform_matrix
from .pipeline_node import PipelineNode, run, LATEST_QOS
from .icp_estimator import IcpEstimator, ICP_DEFAULTS


def matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return transform_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])


class IcpNode(PipelineNode):
    def __init__(self, model_data=None, input_enabled=True, node_name="icp_node", extra_defaults=None, **kwargs):
        defaults = dict(ICP_DEFAULTS)
        defaults.update(extra_defaults or {})
        super().__init__(node_name, defaults, **kwargs)
        self.estimator = IcpEstimator(self.cfg, model_data)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(ObjectDetectionArray, self.cfg['output_topic'], LATEST_QOS)
        self.start_worker(self.estimate)
        self.subscription = (self.create_subscription(DetectionFrame, self.cfg['input_topic'],
                                                      self.receive, LATEST_QOS)
                             if input_enabled else None)

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
        return self.estimator.estimate(payload)


def main(args=None):
    run(IcpNode, args)
