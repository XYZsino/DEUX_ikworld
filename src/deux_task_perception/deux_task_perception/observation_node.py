"""External ZED RGB-D observation boundary. Never publishes robot commands."""
from copy import deepcopy
import math
import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import Buffer, TransformListener, TransformException
from deux_task_msgs.msg import Observation
from .core import stamp_ns, normalize_rgb, normalize_depth, validate_bundle


class ObservationNode(Node):
    def __init__(self):
        super().__init__('observation_node')
        defaults = {
            'rgb_topic': '/zed/zed_node/rgb/color/rect/image',
            'depth_topic': '/zed/zed_node/depth/depth_registered',
            'rgb_info_topic': '/zed/zed_node/rgb/color/rect/camera_info',
            'depth_info_topic': '/zed/zed_node/depth/camera_info',
            'output_topic': '/task/observation',
            'base_frame': 'base_link',
            'width': 640, 'height': 480,
            'sync_queue_size': 10, 'sync_slop_s': .02,
            'max_image_age_s': .5, 'camera_timeout_s': 1.,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.cfg = {name: self.get_parameter(name).value for name in defaults}
        for key in ('sync_slop_s', 'max_image_age_s', 'camera_timeout_s'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        if min(self.cfg['width'], self.cfg['height'], self.cfg['sync_queue_size']) <= 0:
            raise ValueError('positive image dimensions and queue required')
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        qos = qos_profile_sensor_data
        self.publisher = self.create_publisher(Observation, self.cfg['output_topic'], qos)
        self.diagnostics = self.create_publisher(DiagnosticArray, '/task/observation/diagnostics', 10)
        self.filters = [message_filters.Subscriber(self, typ, self.cfg[name], qos_profile=qos)
                        for typ, name in ((Image, 'rgb_topic'), (Image, 'depth_topic'),
                                          (CameraInfo, 'rgb_info_topic'),
                                          (CameraInfo, 'depth_info_topic'))]
        self.sync = message_filters.ApproximateTimeSynchronizer(
            self.filters, self.cfg['sync_queue_size'], self.cfg['sync_slop_s'])
        self.sync.registerCallback(self._images)
        self.sequence = 0
        self.last_stamp = None
        self.last_publish = None
        self.last_clock = self.get_clock().now().nanoseconds
        self.accepted = 0
        self.rejected = 0
        self.status = 'waiting for synchronized RGB, depth and both CameraInfo streams'
        self.last_input_error = ''
        self.timer = self.create_timer(.5, self._diagnostic)
        self.get_logger().info(f"External ZED observation: {self.cfg['width']}x{self.cfg['height']}; no robot command output")

    @staticmethod
    def _image(header, array, encoding):
        msg = Image()
        msg.header = deepcopy(header)
        msg.height, msg.width = array.shape[:2]
        msg.encoding = encoding
        msg.is_bigendian = False
        msg.step = array.strides[0]
        msg.data = array.tobytes()
        return msg

    def _clock_reset(self, now):
        if now < self.last_clock:
            self.last_stamp = self.last_publish = None
            self.status = 'clock moved backwards; observation caches cleared'
        self.last_clock = now

    def _images(self, rgb, depth, rgb_info, depth_info):
        now = self.get_clock().now().nanoseconds
        self._clock_reset(now)
        stamp = stamp_ns(rgb.header)
        try:
            validate_bundle(rgb, depth, rgb_info, depth_info,
                            self.cfg['width'], self.cfg['height'],
                            int(self.cfg['sync_slop_s']*1e9))
            times = [stamp_ns(m.header) for m in (rgb, depth, rgb_info, depth_info)]
            if min(times) <= 0 or min(times) < now-int(self.cfg['max_image_age_s']*1e9) or max(times) > now:
                raise ValueError('stale or future image/calibration timestamp; check ROS clock')
            if self.last_stamp is not None and stamp <= self.last_stamp:
                raise ValueError('repeated or out-of-order image frame')
            rgb_array = normalize_rgb(rgb)
            depth_array, fraction = normalize_depth(depth)
            out = Observation()
            out.header = deepcopy(rgb.header)
            out.sequence = self.sequence
            out.rgb = self._image(rgb.header, rgb_array, 'rgb8')
            out.depth = self._image(depth.header, depth_array, '32FC1')
            out.rgb_info, out.depth_info = rgb_info, depth_info
            out.valid_depth_fraction = fraction
            try:
                out.base_from_camera = self.tf.lookup_transform(
                    self.cfg['base_frame'], rgb.header.frame_id,
                    Time.from_msg(rgb.header.stamp, clock_type=self.get_clock().clock_type))
                out.camera_transform_valid = True
            except TransformException:
                pass  # No invented extrinsics or substitution of old head camera.
            self.publisher.publish(out)
            self.sequence += 1
            self.accepted += 1
            self.last_stamp, self.last_publish = stamp, now
            self.status = ('RGB-D valid; camera_tf=' + str(out.camera_transform_valid)
                           + f', valid_depth={fraction:.3f}')
        except (ValueError, TypeError, OverflowError) as error:
            self.rejected += 1
            self.status = str(error)
            self.last_input_error = str(error)

    def _diagnostic(self):
        now = self.get_clock().now().nanoseconds
        self._clock_reset(now)
        stalled = self.last_publish is None or now-self.last_publish > self.cfg['camera_timeout_s']*1e9
        diag = DiagnosticStatus()
        diag.name = 'deux_task/observation'
        diag.hardware_id = 'external_zed'
        diag.level = DiagnosticStatus.WARN if stalled else DiagnosticStatus.OK
        diag.message = ('No fresh synchronized RGB-D. ' if stalled else '') + self.status
        diag.values = [KeyValue(key=k, value=str(v)) for k, v in
                       [('accepted', self.accepted), ('rejected', self.rejected),
                        ('last_input_error', self.last_input_error)]]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [diag]
        self.diagnostics.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObservationNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
