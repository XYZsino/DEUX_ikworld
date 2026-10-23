"""Shared single-executor scheduling and acquisition-time freshness checks."""
import math
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from .core import stamp_ns
from .perception_core import LatestWorker


class PipelineNode(Node):
    def __init__(self, name, defaults, **kwargs):
        super().__init__(name, **kwargs)
        defaults = dict(defaults, max_age_s=1.0)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        self.positive('max_age_s')
        self.epoch = 0
        self.last_clock = self.get_clock().now().nanoseconds
        self.last_input = None
        self.accepted = self.rejected = 0
        self.status = 'waiting for input'
        self.worker = None
        self.diag = self.create_publisher(DiagnosticArray, f'/task/{name}/diagnostics', 10)
        self.timer = self.create_timer(.02, self._poll)
        self.diag_timer = self.create_timer(.5, self._diagnostic)

    def positive(self, *keys):
        for key in keys:
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')

    def start_worker(self, function):
        self.worker = LatestWorker(lambda job: (job[0], function(job[1])))

    def now(self):
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock:
            self.epoch += 1
            self.last_input = None
            if self.worker:
                self.worker.pending = None
        self.last_clock = now
        return now

    def fresh(self, header, now):
        return 0 < stamp_ns(header) <= now and now-stamp_ns(header) <= self.cfg['max_age_s']*1e9

    def enqueue(self, msg, payload=None):
        now = self.now()
        stamp = stamp_ns(msg.header)
        if not self.fresh(msg.header, now) or (self.last_input is not None and stamp <= self.last_input):
            self.reject('stale, future, duplicate or out-of-order frame')
            return
        self.last_input = stamp
        self.worker.submit((self.epoch, msg if payload is None else payload))

    def reject(self, message):
        self.rejected += 1
        self.status = str(message)

    def _poll(self):
        now = self.now()
        if self.worker is None:
            return
        completed = self.worker.poll()
        if completed is None:
            return
        try:
            epoch, result = completed.result()
            if epoch != self.epoch or not self.fresh(result.header, now):
                raise ValueError('expired result or ROS clock reset during computation')
            self.publisher.publish(result)
            self.accepted += 1
            self.status = 'result published (not hardware verification)'
        except Exception as error:
            self.reject(f'{type(error).__name__}: {error}')

    def _diagnostic(self):
        status = DiagnosticStatus()
        status.name = f'deux_task/{self.get_name()}'
        status.level = DiagnosticStatus.WARN
        status.message = self.status
        status.values = [KeyValue(key=k, value=str(v)) for k, v in
                         [('published', self.accepted), ('rejected', self.rejected),
                          ('queue_replaced', self.worker.dropped if self.worker else 0)]]
        out = DiagnosticArray()
        out.header.stamp = self.get_clock().now().to_msg()
        out.status = [status]
        self.diag.publish(out)

    def destroy_node(self):
        if self.worker:
            self.worker.close()
        return super().destroy_node()


def run(node_type, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = node_type()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
