"""Latest arm/normalized-hand measurements, without a BT or state relay."""
from threading import Lock
import numpy as np
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy
from deux_task_core.model import ARM_NAMES, HAND_NAMES, JOINT_NAMES
from .common import current, seconds, joint_array


class JointFeedback:
    def __init__(self, node):
        self.node = node
        self.lock = Lock()
        self.samples = {}
        self.last_clock = node.now()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subs = [node.create_subscription(
            JointState, topic, lambda msg, s=side: self.receive(s, msg), qos)
            for side, topic in (('arm', '/arm/joint_states'), ('hand', '/task/hand_state'))]

    def check_clock(self):
        now = self.node.now()
        if now < self.last_clock:
            self.samples.clear()
        self.last_clock = now
        return now

    def receive(self, side, msg):
        with self.lock:
            now = self.check_clock()
            if not current(msg.header.stamp, now, self.node.config['max_joint_age']):
                return
            old = self.samples.get(side)
            if old and seconds(msg.header.stamp) <= old[0]:
                return
            try:
                values = joint_array(msg, ARM_NAMES if side == 'arm' else HAND_NAMES)
            except (ValueError, KeyError):
                # A fresh malformed packet must not leave an older valid one usable.
                self.samples[side] = (seconds(msg.header.stamp), None)
                return
            self.samples[side] = (seconds(msg.header.stamp), values)

    def measured(self):
        with self.lock:
            now = self.check_clock()
            values = {}
            for side, names in (('arm', ARM_NAMES), ('hand', HAND_NAMES)):
                sample = self.samples.get(side)
                if (sample is None or sample[1] is None
                        or not 0 <= now - sample[0] <= self.node.config['max_joint_age']):
                    raise ValueError(f'{side} measured joints unavailable/stale')
                values.update(zip(names, sample[1]))
            return np.array([values[name] for name in JOINT_NAMES])
