"""Reader String receipt with bounded, target-specific scan session association."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from deux_task_msgs.msg import BarcodeRead, ScanSession


class BarcodeBridgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__("barcode_bridge_node", **kwargs)
        self.declare_parameter("input_topic", "/barcode/raw")
        self.declare_parameter("simulated", False)
        self.session = None
        self.last_clock = 0.0
        self.seen = set()
        self.publisher = self.create_publisher(BarcodeRead, "/task/barcode_read", 10)
        self.sub = self.create_subscription(
            String, self.get_parameter("input_topic").value, self.read, 10
        )
        self.session_sub = self.create_subscription(
            ScanSession,
            "/task/scan_session",
            self.set_session,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def set_session(self, msg):
        self.session = msg
        self.seen.clear()

    def read(self, msg):
        now = self.get_clock().now()
        t = now.nanoseconds * 1e-9
        if t < self.last_clock:
            self.session = None
        self.last_clock = t
        s = self.session
        value = msg.data.strip()
        sec = lambda stamp: stamp.sec + stamp.nanosec * 1e-9
        if (
            s is None
            or not s.active
            or not value
            or len(value) > 4096
            or value in self.seen
        ):
            return
        if not sec(s.header.stamp) <= t <= sec(s.expires_at):
            return
        out = BarcodeRead()
        out.header.stamp = now.to_msg()
        out.value = value
        out.session_id = s.session_id
        out.track_id = s.track_id
        out.simulated = self.get_parameter("simulated").value
        self.publisher.publish(out)
        self.seen.add(value)


def main(args=None):
    rclpy.init(args=args)
    node = BarcodeBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
