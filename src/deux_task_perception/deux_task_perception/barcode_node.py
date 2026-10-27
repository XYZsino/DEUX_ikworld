"""Newland serial + BT scan sessions in one ROS node; SIM topic input is explicit."""
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from deux_task_msgs.msg import BarcodeRead, ScanSession
from .barcode_serial import BarcodeController, Session, DEFAULT_PATTERN, connect


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class BarcodeNode(Node):
    def __init__(self, *, connector=None, **kwargs):
        super().__init__('barcode_node', **kwargs)
        defaults = dict(enabled=False, simulated=False, port='', serial_number='', baudrate=115200,
                        input_topic='/barcode/raw', value_pattern=DEFAULT_PATTERN,
                        quiet_s=.2, drain_timeout_s=1., scan_timeout_s=5., reconnect_s=1., max_bytes=4096)
        for key, value in defaults.items():
            self.declare_parameter(key, value, ParameterDescriptor(read_only=True))
        c = {k: self.get_parameter(k).value for k in defaults}
        if c['baudrate'] <= 0:
            raise ValueError('positive baudrate required')
        self.publisher = self.create_publisher(BarcodeRead, '/task/barcode_read', 10)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/task/barcode/diagnostics', 10)
        self.controller = BarcodeController(self.emit,
            connector=connector or (lambda: connect(c['port'], c['serial_number'], c['baudrate'])),
            simulated=c['simulated'], enabled=c['enabled'], pattern=c['value_pattern'],
            quiet_s=c['quiet_s'], drain_timeout_s=c['drain_timeout_s'], scan_timeout_s=c['scan_timeout_s'],
            reconnect_s=c['reconnect_s'], max_bytes=c['max_bytes'])
        self.session_sub = self.create_subscription(ScanSession, '/task/scan_session', self.set_session,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.raw_sub = None
        if c['simulated']:
            self.raw_sub = self.create_subscription(String, c['input_topic'], self.read_simulated, 10)
        self.last_diagnostic = None
        self.diagnostic_at = 0.
        # A paused ROS clock must not defeat the REAL serial timeout/watchdog.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.02, self.poll, clock=self.steady_clock)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_session(self, msg):
        self.controller.request(Session(msg.session_id, msg.track_id,
            seconds(msg.header.stamp), seconds(msg.expires_at)), msg.active, self.now(), time.monotonic())
        self.publish_diagnostic()

    def read_simulated(self, msg):
        self.controller.check_clock(self.now(), time.monotonic())
        self.controller.value(msg.data.encode('utf-8'), self.now(), time.monotonic())

    def emit(self, session, value, received_at):
        out = BarcodeRead(value=value, session_id=session.identity, track_id=session.track_id,
                          simulated=self.controller.simulated)
        ns = int(round(received_at * 1e9))
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(ns, 10**9)
        self.publisher.publish(out)

    def poll(self):
        self.controller.tick(self.now(), time.monotonic())
        self.publish_diagnostic()

    def publish_diagnostic(self):
        c = self.controller
        signature = (c.state, c.reason, c.session.identity if c.session else '')
        now = time.monotonic()
        if signature == self.last_diagnostic and now - self.diagnostic_at < 1.:
            return
        self.last_diagnostic, self.diagnostic_at = signature, now
        level = (DiagnosticStatus.ERROR if c.state in ('error', 'disconnected') else
                 DiagnosticStatus.WARN if c.state in ('disabled', 'timeout', 'canceled') else DiagnosticStatus.OK)
        status = DiagnosticStatus(name='deux_task/barcode', hardware_id='sim' if c.simulated else 'Newland-HR2180-BT',
            level=level, message=f'{c.state}: {c.reason}',
            values=[KeyValue(key='session_id', value=signature[2]),
                    KeyValue(key='connected', value=str(c.simulated or c.port is not None))])
        msg = DiagnosticArray(status=[status]); msg.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics.publish(msg)

    def destroy_node(self):
        try:
            self.controller.close()
        except Exception as error:
            self.get_logger().error(f'scanner shutdown stop failed: {error}')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BarcodeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
