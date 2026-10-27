"""Real ROS messages + pyserial on a Linux pseudo-terminal; no hardware access."""
import os
import time
import pytest
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('serial')
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray
from deux_task_msgs.msg import ScanSession, BarcodeRead
from deux_task_perception.barcode_node import BarcodeNode
from deux_task_perception.barcode_serial import START, STOP


def stamp(node, offset=0.):
    from builtin_interfaces.msg import Time
    ns = node.get_clock().now().nanoseconds + round(offset * 1e9)
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


def test_ros_serial_trigger_read_cancel_timeout_disconnect_and_no_raw_subscription():
    master, slave = os.openpty(); os.set_blocking(master, False)
    path = os.ttyname(slave)
    rclpy.init(domain_id=219)
    source = rclpy.create_node('serial_scanner_test')
    node = BarcodeNode(parameter_overrides=[Parameter(k, value=v) for k, v in
        dict(enabled=True, port=path, quiet_s=.06, drain_timeout_s=.3, scan_timeout_s=.3, reconnect_s=.1).items()])
    ex = SingleThreadedExecutor(); ex.add_node(source); ex.add_node(node)
    reads, diagnostics, commands = [], [], bytearray()
    source.create_subscription(BarcodeRead, '/task/barcode_read', reads.append, 10)
    source.create_subscription(DiagnosticArray, '/task/barcode/diagnostics', diagnostics.append, 10)
    pub = source.create_publisher(ScanSession, '/task/scan_session',
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin(predicate, timeout=3.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            ex.spin_once(timeout_sec=.01)
            if master >= 0:
                try:commands.extend(os.read(master, 8192))
                except BlockingIOError:pass
            if predicate():return
        raise AssertionError(f'timeout: {node.controller.state}: {node.controller.reason}')
    def session(identity, ttl=2.):
        msg = ScanSession(session_id=identity, track_id='bag-7', active=True,
                          expires_at=stamp(source, ttl)); msg.header.stamp = stamp(source)
        pub.publish(msg)
        return msg
    try:
        spin(lambda: node.controller.state == 'idle' and pub.get_subscription_count() > 0)
        assert '/barcode/raw' not in {s.topic_name for s in node.subscriptions}
        a = session('a')
        spin(lambda: node.controller.state == 'scanning')
        assert START in commands
        os.write(master, b'\x06\rOK\r000123')
        spin(lambda: bool(node.controller.buffer))
        assert not reads
        os.write(master, b'4567890\r\n')
        spin(lambda: bool(reads))
        assert reads[-1].value == '0001234567890'
        assert reads[-1].session_id == 'a' and reads[-1].track_id == 'bag-7'
        assert not reads[-1].simulated and (reads[-1].header.stamp.sec, reads[-1].header.stamp.nanosec) >= (a.header.stamp.sec, a.header.stamp.nanosec)
        assert node.controller.state == 'read' and commands.endswith(STOP)
        a.active = False; pub.publish(a)
        b = session('b')
        spin(lambda: node.controller.state == 'draining')
        os.write(master, b'2087231250396\r')  # late old bytes during pre-trigger drain
        spin(lambda: node.controller.state == 'scanning')
        assert len(reads) == 1
        b.active = False; pub.publish(b)
        spin(lambda: node.controller.state == 'canceled')
        assert commands.endswith(STOP)
        session('c'); spin(lambda: node.controller.state == 'scanning')
        spin(lambda: node.controller.state == 'timeout')
        spin(lambda: any('timeout:' in x.status[0].message for x in diagnostics))
        assert len(reads) == 1 and commands.endswith(STOP)
        session('d'); spin(lambda: node.controller.state == 'scanning')
        os.close(master); master = -1
        spin(lambda: node.controller.state == 'disconnected')
        assert node.controller.session is None and len(reads) == 1
    finally:
        ex.shutdown(); node.destroy_node(); source.destroy_node(); rclpy.shutdown()
        if master >= 0:os.close(master)
        os.close(slave)
