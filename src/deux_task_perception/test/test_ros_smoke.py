"""Run after colcon build on a ROS 2 host; no camera or actuator needed."""
import time
import pytest
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('deux_task_msgs.msg')
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from deux_task_msgs.msg import Observation
from deux_task_perception.observation_node import ObservationNode


def test_real_ros_transport_rgbd_without_urdf_or_joint_feedback():
    rclpy.init()
    node = ObservationNode()
    source = rclpy.create_node('synthetic_zed_test')
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    ex.add_node(source)
    pubs = [source.create_publisher(typ, node.cfg[name], qos_profile_sensor_data)
            for typ,name in ((Image,'rgb_topic'),(Image,'depth_topic'),
                             (CameraInfo,'rgb_info_topic'),(CameraInfo,'depth_info_topic'))]
    received = []
    sub = source.create_subscription(Observation, '/task/observation', received.append,
                                     qos_profile_sensor_data)
    try:
        deadline = time.monotonic()+6
        while time.monotonic() < deadline and not received:
            stamp = source.get_clock().now().to_msg()
            rgb, dep = Image(), Image()
            for m in (rgb,dep):
                m.header.stamp = stamp
                m.header.frame_id = 'new_upper_camera_optical'
                m.width,m.height = 640,480
            rgb.encoding,rgb.step,rgb.data = 'bgra8',2560, bytes([1,2,3,255])*(640*480)
            dep.encoding,dep.step,dep.data = 'mono16',1280, b'\xe8\x03'*(640*480)
            infos = [CameraInfo(),CameraInfo()]
            for m in infos:
                m.header = rgb.header
                m.width,m.height = 640,480
                m.k = [500.,0.,320.,0.,500.,240.,0.,0.,1.]
                m.p = [500.,0.,320.,0.,0.,500.,240.,0.,0.,0.,1.,0.]
            for pub,msg in zip(pubs,[rgb,dep,*infos]): pub.publish(msg)
            for _ in range(8): ex.spin_once(timeout_sec=.02)
        assert received, node.status
        out = received[-1]
        assert out.rgb.encoding == 'rgb8' and out.depth.encoding == '32FC1'
        assert bytes(out.rgb.data[:3]) == bytes([3,2,1])
        assert not out.camera_transform_valid
        assert not hasattr(out, "joints") and not hasattr(out, "objects")
        topics = {s.topic_name for s in node.subscriptions}
        assert not topics & {"/arm/joint_states", "/task/hand_state", "/task/object_detections"}
        assert out.header.frame_id == 'new_upper_camera_optical'
        assert out.valid_depth_fraction == 1.
    finally:
        source.destroy_subscription(sub)
        ex.shutdown()
        source.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
