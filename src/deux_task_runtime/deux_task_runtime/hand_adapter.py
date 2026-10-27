"""Hand protocol adapter owned by ExecutionNode; no ROS action/server of its own."""
from pathlib import Path
import numpy as np
import yaml
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from deux_task_core.model import HAND_NAMES, vector
from .common import current, seconds


class HandAdapter:
    def __init__(self, node):
        self.node = node
        self.mapping = None
        self.raw = {}
        self.command_pubs = {}
        self.feedback_subs = []
        self.publisher = node.create_publisher(JointState, "/task/hand_state", qos_profile_sensor_data)
        path = node.get_parameter("mapping_file").value
        if not path:
            return
        self.mapping = yaml.safe_load(Path(path).read_text())
        self.validate_mapping()
        for side in ("left", "right"):
            cfg = self.mapping[side]
            self.feedback_subs.append(node.create_subscription(
                get_message(cfg["feedback_type"]), cfg["feedback_topic"],
                lambda msg, s=side: self.feedback(s, msg), qos_profile_sensor_data))
        if node.get_parameter("hardware_enabled").value:
            from deux_hand_msgs.msg import HandCtlMessage, MotorData
            self.Command, self.Motor = HandCtlMessage, MotorData
            self.command_pubs = {s: node.create_publisher(HandCtlMessage, f"/hand/{s}/cmd", 10)
                                 for s in ("left", "right")}

    def validate_mapping(self):
        if (
            self.mapping.get("calibrated") is not True
            or self.mapping.get("position_mode_confirmed") is not True
        ):
            raise ValueError(
                "hand calibration/position protocol must be explicitly confirmed"
            )
        for side in ("left", "right"):
            c = self.mapping[side]
            names = [n for n in HAND_NAMES if n.startswith(side)]
            if set(c["joints"]) != set(names):
                raise ValueError("exact hand joint mapping required")
            ids = [c["joints"][n]["id"] for n in names]
            if len(set(ids)) != 7 or any(
                type(i) != int or not 0 <= i <= 255 for i in ids
            ):
                raise ValueError("invalid motor IDs")
            for key in ("mode", "action_state", "action_state_save"):
                if type(c[key]) != int or not 0 <= c[key] <= 255:
                    raise ValueError("explicit hand protocol byte required")
            if c["feedback_type"] not in (
                "sensor_msgs/msg/JointState",
                "deux_hand_msgs/msg/HandCtlMessage",
            ):
                raise ValueError("unsupported measured feedback protocol")
            for name, m in c["joints"].items():
                if (
                    m["sign"] not in (-1, 1)
                    or not np.isfinite(
                        [
                            m["offset"],
                            m["min"],
                            m["max"],
                            m["velocity"],
                            m["goal_current"],
                        ]
                    ).all()
                ):
                    raise ValueError("invalid hand calibration")
                if m["min"] >= m["max"] or m["velocity"] <= 0:
                    raise ValueError("invalid hand limits")

    def feedback(self, side, msg):
        try:
            if self.node.config is None or not current(
                msg.header.stamp, self.node.now(), self.node.config["max_joint_age"]
            ):
                return
            c = self.mapping[side]
            if isinstance(msg, JointState):
                if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(
                    msg.name
                ):
                    raise ValueError("malformed hand feedback")
                raw = dict(zip(msg.name, msg.position))
                values = {
                    n: (raw[m["feedback_name"]] - m["offset"]) / m["sign"]
                    for n, m in c["joints"].items()
                }
            else:
                if len({m.id for m in msg.motors}) != len(msg.motors) or any(
                    m.hardware_error_status for m in msg.motors
                ):
                    raise ValueError("motor error or duplicate feedback ID")
                raw = {m.id: m.rad for m in msg.motors}
                values = {
                    n: (raw[m["id"]] - m["offset"]) / m["sign"]
                    for n, m in c["joints"].items()
                }
            vector(list(values.values()), 7)
            if any(
                not c["joints"][n]["min"] <= value <= c["joints"][n]["max"]
                for n, value in values.items()
            ):
                raise ValueError("measured hand position outside calibrated limits")
            self.raw[side] = (msg.header, values)
            if all(
                s in self.raw
                and current(
                    self.raw[s][0].stamp, self.node.now(), self.node.config["max_joint_age"]
                )
                for s in ("left", "right")
            ):
                out = JointState()
                out.header = min(
                    (v[0] for v in self.raw.values()), key=lambda h: seconds(h.stamp)
                )
                out.name = list(HAND_NAMES)
                out.position = self.measured_hand().tolist()
                self.publisher.publish(out)
        except (ValueError, KeyError) as error:
            self.raw.pop(side, None)
            self.node.get_logger().warning(str(error))

    def measured_hand(self):
        if self.node.config is None or not all(
            s in self.raw
            and current(self.raw[s][0].stamp, self.node.now(), self.node.config["max_joint_age"])
            for s in ("left", "right")
        ):
            raise ValueError("hand feedback stale/missing")
        values = {k: v for _, d in self.raw.values() for k, v in d.items()}
        return vector([values[n] for n in HAND_NAMES], 14)

    def validate(self, t, q):
        if not self.command_pubs or self.mapping is None:
            raise ValueError("hand dispatch disabled/unconfigured")
        for i, name in enumerate(HAND_NAMES):
            m = self.mapping[name.split("_")[0]]["joints"][name]
            if np.any(q[:, i] < m["min"]) or np.any(q[:, i] > m["max"]):
                raise ValueError("hand joint limit")
            if np.any(abs(np.diff(q[:, i]) / np.diff(t)) > m["velocity"]):
                raise ValueError("hand velocity limit")

    def command(self, positions):
        for side, pub in self.command_pubs.items():
            cfg = self.mapping[side]
            msg = self.Command()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.side = side
            for key in ("mode", "action_state", "action_state_save"):
                setattr(msg, key, cfg[key])
            for name, m in cfg["joints"].items():
                motor = self.Motor()
                motor.id = m["id"]
                motor.torque = True
                motor.rad = float(
                    m["sign"] * positions[HAND_NAMES.index(name)] + m["offset"]
                )
                motor.velocity = float(m["velocity"])
                motor.goal_current = float(m["goal_current"])
                msg.motors.append(motor)
            pub.publish(msg)


    def hold_measured(self):
        # Never guess a hold angle when feedback is invalid. Successful close
        # must retain the commanded preload rather than stalled measured angles.
        try:
            self.command(self.measured_hand())
        except (ValueError, KeyError):
            self.node.get_logger().error("feedback lost: no guessed hold; driver watchdog required")
