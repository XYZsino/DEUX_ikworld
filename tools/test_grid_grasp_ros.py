"""Isolated-domain synthetic ROS linkage and disabled-execution test; no hardware."""
import json
import os
import time
if os.environ.get('ROS_DOMAIN_ID') in (None, '0', '12'):
    raise RuntimeError('Use a separate ROS_DOMAIN_ID, e.g. 217; never inject synthetic observations on hardware domain 12')
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from deux_task_msgs.msg import ObjectDetectionArray, ObjectDetection
from deux_task_msgs.action import MoveToReady
from deux_task_core.model import ARM_NAMES
from deux_task_runtime.common import ros_time
if os.environ.get('DEUX_TEST_CLASSIFIED_GRID')=='1':
    from classified_grasp_trial import ClassifiedTrial as GridTrial
else:
    from grid_grasp_trial import GridTrial


def main():
    rclpy.init(); runtime = GridTrial()
    runtime.out = runtime.out.parent/'grid-grasp-tests/synthetic-preview'; runtime.out.mkdir(parents=True, exist_ok=True)
    producer = Node('grid_synthetic_test'); executor = SingleThreadedExecutor()
    executor.add_node(runtime); executor.add_node(producer)
    predictions = producer.create_publisher(ObjectDetectionArray, '/task/prediction_detections', qos_profile_sensor_data)
    arms = producer.create_publisher(JointState, '/arm/joint_states', qos_profile_sensor_data)
    tracking = producer.create_publisher(String, '/task/icp_preview_status', 10)
    calibration = producer.create_publisher(String, '/task/camera_calibration_status', 10)
    result = {}; started = time.monotonic()
    def publish():
        now = producer.get_clock().now().nanoseconds/1e9
        calibration.publish(String(data=json.dumps(dict(valid=True))))
        tracking.publish(String(data=json.dumps(dict(tracking=True, target_id='grid-synthetic'))))
        msg = ObjectDetectionArray(); msg.header.frame_id = 'base_link'; msg.header.stamp = ros_time(now)
        msg.model_id = runtime.config['model_id']; msg.sequence = round(now*1000)
        d = ObjectDetection(); d.track_id = 'grid-synthetic'; d.confidence = 1.; d.velocity_valid = True
        d.velocity.linear.y = -.09; d.pose.pose.orientation.w = 1.
        d.pose.pose.position.x = .48; d.pose.pose.position.y = .60-.09*(time.monotonic()-started); d.pose.pose.position.z = .97
        msg.detections = [d]; predictions.publish(msg)
        arm = JointState(); arm.header.stamp = ros_time(now); arm.name = list(ARM_NAMES)
        arm.position = [*runtime.library.ready[:7], *runtime.library.ready[14:21]]; arm.velocity = [0.]*14
        arms.publish(arm)
    timer = producer.create_timer(.05, publish)
    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
            if runtime.preview.get('dispatch_ready'): break
        assert runtime.preview.get('dispatch_ready'), runtime.preview
        result['synthetic_pose_to_grid'] = runtime.preview
        client = ActionClient(producer, MoveToReady, '/task/grid_grasp')
        future = client.send_goal_async(MoveToReady.Goal()); deadline = time.monotonic()+3
        while not future.done() and time.monotonic() < deadline: executor.spin_once(timeout_sec=.02)
        assert future.done() and not future.result().accepted, 'preview-only action must reject execution'
        command_topics = ['/arm/deux_arm_controller/joint_trajectory', '/hand/right/grip', '/hand/right/cmd']
        counts = {topic:producer.count_publishers(topic) for topic in command_topics}
        assert not any(counts.values()), counts
        result.update(preview_action_rejected=True, hardware_command_publishers=counts, synthetic_only=True)
        print(json.dumps(result, indent=2))
    finally:
        producer.destroy_timer(timer); executor.remove_node(runtime); executor.remove_node(producer)
        runtime.destroy_node(); producer.destroy_node(); executor.shutdown(); rclpy.shutdown()


if __name__ == '__main__': main()
