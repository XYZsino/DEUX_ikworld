"""Read-only hardware-domain probe. Saves camera/pose linkage evidence, no commands."""
import json
import time
from pathlib import Path
from collections import Counter
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action.graph import get_action_client_names_and_types_by_node
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from deux_task_msgs.msg import DetectionArray, ObjectDetectionArray


def main():
    rclpy.init(); node = Node('grid_grasp_readonly_probe')
    root = Path(__file__).resolve().parents[1]; out = root/'.setup/grid-grasp-test'; out.mkdir(parents=True, exist_ok=True)
    counts = Counter(); latest = {}; frame_saved = [False]
    def string(name):
        def receive(msg):
            counts[name] += 1
            try: latest[name] = json.loads(msg.data)
            except ValueError: latest[name] = msg.data
        return receive
    for topic, name in [('/task/grid_grasp/preview', 'grid_preview'), ('/task/icp_preview_status', 'tracking'),
                        ('/task/camera_calibration_status', 'camera_calibration')]:
        node.create_subscription(String, topic, string(name), 10)
    def detections(msg):
        counts['detections'] += 1
        latest['detections'] = [dict(track_id=b.track_id, class_name=b.class_name, confidence=float(b.confidence)) for b in msg.boxes]
    node.create_subscription(DetectionArray, '/task/detections', detections, qos_profile_sensor_data)
    def prediction(msg):
        counts['prediction_arrays'] += 1
        latest['predictions'] = [dict(track_id=d.track_id, velocity_valid=d.velocity_valid,
            position_m=[d.pose.pose.position.x, d.pose.pose.position.y, d.pose.pose.position.z],
            velocity_m_s=[d.velocity.linear.x, d.velocity.linear.y, d.velocity.linear.z]) for d in msg.detections]
    node.create_subscription(ObjectDetectionArray, '/task/prediction_detections', prediction, qos_profile_sensor_data)
    node.create_subscription(JointState, '/arm/joint_states', lambda msg: counts.update(['arm_feedback']), qos_profile_sensor_data)
    def image(msg):
        counts['camera_rgb'] += 1
        if not frame_saved[0]:
            try:
                from cv_bridge import CvBridge
                import cv2
                import numpy as np
                rgb = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
                roi = json.loads((root/'.setup/single-track-roi.json').read_text())
                cv2.polylines(rgb, [np.rint(roi['points']).astype(np.int32)], True, (0, 210, 255), 2)
                cv2.imwrite(str(out/'live-roi.jpg'), rgb); frame_saved[0] = True
            except Exception as error: latest['image_error'] = str(error)
    node.create_subscription(Image, '/zed/color/image_raw', image, qos_profile_sensor_data)
    began = time.monotonic()
    try:
        while time.monotonic()-began < 15: rclpy.spin_once(node, timeout_sec=.1)
        grid_commands = {}
        for topic in ['/arm/deux_arm_controller/joint_trajectory', '/hand/right/grip', '/hand/right/cmd']:
            grid_commands[topic] = [i.node_name for i in node.get_publishers_info_by_topic(topic) if i.node_name == 'grid_grasp_trial']
        record = dict(actual_hardware_streams=True, motion_requested=False, elapsed_s=time.monotonic()-began,
                      received=dict(counts), latest=latest, grid_command_publishers=grid_commands,
                      grid_action_clients=(get_action_client_names_and_types_by_node(node, 'grid_grasp_trial', '/')
                          if 'grid_grasp_trial' in node.get_node_names() else []),
                      frame_saved=frame_saved[0])
        if any(grid_commands.values()) or record['grid_action_clients']: raise RuntimeError('preview unexpectedly has hardware command capability')
        (out/'live-probe.json').write_text(json.dumps(record, indent=2)); print(json.dumps(record, indent=2))
    finally: node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
