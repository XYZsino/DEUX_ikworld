"""Aggregate fresh robot state and pose results without subscribing to images."""
from copy import deepcopy
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import Buffer, TransformListener, TransformException
from deux_task_msgs.msg import StateObservation, ObjectDetectionArray
from .core import History, ARM_NAMES, JOINT_NAMES, joint_values, validate_objects, stamp_ns


class StateObservationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('state_observation_node', **kwargs)
        defaults = {'base_frame': 'base_link', 'camera_frame': '',
                    'left_ee_frame': '', 'right_ee_frame': '',
                    'arm_state_topic': '/arm/joint_states', 'hand_state_topic': '/task/hand_state',
                    'objects_topic': '/task/object_detections',
                    'output_topic': '/task/state_observation',
                    'publish_rate_hz': 10.0, 'max_joint_age_s': .1, 'max_object_age_s': .6}
        for key, value in defaults.items(): self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        for key in ('publish_rate_hz', 'max_joint_age_s', 'max_object_age_s'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be positive and finite')
        self.arm, self.hand, self.objects = History(), History(), History()
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.last_clock = self.get_clock().now().nanoseconds
        self.sequence = 0; self.rejected = 0
        qos = qos_profile_sensor_data
        self.publisher = self.create_publisher(StateObservation, self.cfg['output_topic'], qos)
        self.diag = self.create_publisher(DiagnosticArray, '/task/state_observation/diagnostics', 10)
        self.subs = [self.create_subscription(JointState, self.cfg['arm_state_topic'], self.arm.add, qos),
                     self.create_subscription(JointState, self.cfg['hand_state_topic'], self.hand.add, qos),
                     self.create_subscription(ObjectDetectionArray, self.cfg['objects_topic'], self.receive_objects, qos)]
        self.timer = self.create_timer(1/self.cfg['publish_rate_hz'], self.publish_state)

    def check_clock(self):
        now = self.get_clock().now().nanoseconds
        if now < self.last_clock:
            for h in (self.arm, self.hand, self.objects): h.clear()
        self.last_clock = now
        return now

    def receive_objects(self, msg):
        now = self.check_clock()
        try:
            validate_objects(msg, self.cfg['base_frame'])
            if not 0 < stamp_ns(msg.header) <= now:
                raise ValueError('invalid object acquisition time')
            self.objects.add(msg)
            # Forward a new acquired pose immediately; the periodic tick alone
            # adds avoidable latency at the camera/control phase boundary.
            self.publish_state()
            self.timer.reset()
        except ValueError:
            self.rejected += 1

    def publish_state(self):
        now = self.check_clock()
        out = StateObservation()
        out.header.stamp = Time(nanoseconds=now, clock_type=self.get_clock().clock_type).to_msg()
        out.header.frame_id = self.cfg['base_frame']
        out.sequence = self.sequence; self.sequence += 1
        objects = self.objects.at(now, int(self.cfg['max_object_age_s']*1e9))
        if objects is not None:
            out.objects = objects  # Preserve original acquisition header and sequence.
            out.objects_valid = out.perception_fresh = True
        age = int(self.cfg['max_joint_age_s']*1e9)
        arm, hand = self.arm.at(now, age), self.hand.at(now, age)
        out.joints_reason = "arm/hand feedback missing or stale"
        if arm is not None and hand is not None:
            try:
                hand_names = [n for n in JOINT_NAMES if n not in ARM_NAMES]
                values = dict(zip(ARM_NAMES, joint_values(arm, ARM_NAMES)))
                values.update(zip(hand_names, joint_values(hand, hand_names)))
                out.joints_acquired_at = deepcopy(min((arm, hand), key=lambda m: stamp_ns(m.header)).header.stamp)
                out.joints_valid = True
                out.joints_reason = ""
            except (ValueError, KeyError) as error:
                out.joints_reason = f"invalid joint feedback: {error}"
                self.rejected += 1
        when = Time.from_msg(out.header.stamp, clock_type=self.get_clock().clock_type)
        for cfg_key, field, valid in [('camera_frame', 'base_from_camera', 'camera_transform_valid'),
                                      ('left_ee_frame', 'base_from_left_ee', 'left_ee_valid'),
                                      ('right_ee_frame', 'base_from_right_ee', 'right_ee_valid')]:
            if self.cfg[cfg_key]:
                try:
                    setattr(out, field, self.tf.lookup_transform(self.cfg['base_frame'], self.cfg[cfg_key], when))
                    setattr(out, valid, True)
                except TransformException: pass
        self.publisher.publish(out)
        if self.sequence % max(1, round(self.cfg['publish_rate_hz'])) == 0:
            status = DiagnosticStatus(name='deux_task/state_observation',
                                      level=DiagnosticStatus.OK if out.objects_valid else DiagnosticStatus.WARN,
                                      message=f'objects={out.objects_valid}, joints={out.joints_valid}; no image payloads',
                                      values=[KeyValue(key='rejected', value=str(self.rejected))])
            self.diag.publish(DiagnosticArray(header=out.header, status=[status]))


def main(args=None):
    rclpy.init(args=args); node = StateObservationNode()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
