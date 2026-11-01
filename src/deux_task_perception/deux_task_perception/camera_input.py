"""Consumer-local RGB-D synchronization, with no image copy or publication."""
from types import SimpleNamespace
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data
from .core import stamp_ns, validate_bundle


class CameraInput:
    def __init__(self, node, callback, *, width=640, height=360):
        defaults = dict(rgb_topic='/zed/color/image_raw', depth_topic='/zed/depth/image_raw',
                        rgb_info_topic='/zed/color/camera_info', depth_info_topic='/zed/depth/camera_info',
                        camera_sync_queue_size=10, camera_sync_slop_s=.02)
        for key, value in defaults.items():
            if not node.has_parameter(key):
                node.declare_parameter(key, value)
        self.node, self.callback = node, callback
        self.width, self.height = width, height
        self.slop = node.get_parameter('camera_sync_slop_s').value
        size = node.get_parameter('camera_sync_queue_size').value
        if size < 1 or not 0 < self.slop <= .1:
            raise ValueError('invalid camera synchronization settings')
        self.accepted = self.rejected = 0
        self.last_error = ''
        self.filters = [message_filters.Subscriber(node, typ, node.get_parameter(key).value,
                        qos_profile=qos_profile_sensor_data)
                        for key, typ in (('rgb_topic', Image), ('depth_topic', Image),
                                         ('rgb_info_topic', CameraInfo), ('depth_info_topic', CameraInfo))]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.filters, size, self.slop)
        self.sync.registerCallback(self.receive)

    def receive(self, rgb, depth, rgb_info, depth_info):
        try:
            frame = self.bundle(rgb, depth, rgb_info, depth_info, self.width, self.height, self.slop)
        except (ValueError, TypeError, OverflowError) as exc:
            self.rejected += 1
            self.last_error = str(exc)
            return
        self.accepted += 1
        self.callback(frame)

    @staticmethod
    def bundle(rgb, depth, rgb_info, depth_info, width, height, slop):
        validate_bundle(rgb, depth, rgb_info, depth_info, width, height, int(slop*1e9))
        identity = stamp_ns(rgb.header)
        if identity <= 0:
            raise ValueError('invalid RGB acquisition timestamp')
        # Same identity in independent consumers, even when their frame drops differ.
        return SimpleNamespace(header=rgb.header, sequence=identity, rgb=rgb, depth=depth,
                               rgb_info=rgb_info, depth_info=depth_info,
                               camera_transform_valid=False, base_from_camera=None)


class RgbCameraInput:
    """GUI consumes only RGB and its calibration; geometry arrives as metadata."""
    def __init__(self, node, callback):
        self.callback = callback
        for key, value in dict(rgb_topic='/zed/color/image_raw', rgb_info_topic='/zed/color/camera_info',
                               camera_sync_queue_size=10, camera_sync_slop_s=.02).items():
            if not node.has_parameter(key):node.declare_parameter(key,value)
        self.slop = node.get_parameter('camera_sync_slop_s').value
        self.filters = [message_filters.Subscriber(node, typ, node.get_parameter(key).value,
                        qos_profile=qos_profile_sensor_data)
                        for key,typ in [('rgb_topic',Image),('rgb_info_topic',CameraInfo)]]
        self.sync = message_filters.ApproximateTimeSynchronizer(self.filters,
            node.get_parameter('camera_sync_queue_size').value,self.slop)
        self.sync.registerCallback(self.receive)

    def receive(self, rgb, info):
        if (stamp_ns(rgb.header)<=0 or rgb.header.frame_id!=info.header.frame_id
                or not rgb.header.frame_id or (rgb.width,rgb.height)!=(info.width,info.height)
                or abs(stamp_ns(rgb.header)-stamp_ns(info.header))>self.slop*1e9):
            return
        self.callback(SimpleNamespace(header=rgb.header,sequence=stamp_ns(rgb.header),rgb=rgb,
            rgb_info=info,depth=None,depth_info=None,camera_transform_valid=False,base_from_camera=None))
