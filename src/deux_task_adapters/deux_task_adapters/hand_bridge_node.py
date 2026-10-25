"""Explicit motor mapping; only measured feedback is published as JointState."""

from pathlib import Path
import time
import numpy as np
import yaml
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from deux_task_msgs.action import BridgeMotion
from deux_task_core.model import HAND_NAMES, vector
from deux_task_runtime.common import (
    RuntimeNode,
    ExclusiveAction,
    current,
    seconds,
    pause,
    run,
)
from deux_task_runtime.trajectory import arrays, sample


class HandBridgeNode(RuntimeNode):
    def __init__(self, **kwargs):
        super().__init__("hand_bridge_node", **kwargs)
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("mapping_file", "")
        self.mapping = None
        self.raw = {}
        self.command_pubs = {}
        self.feedback_subs = []
        self.publisher = self.create_publisher(
            JointState, "/task/hand_state", qos_profile_sensor_data
        )
        path = self.get_parameter("mapping_file").value
        if path:
            self.mapping = yaml.safe_load(Path(path).read_text())
            self.validate_mapping()
            for side in ("left", "right"):
                cfg = self.mapping[side]
                typ = get_message(cfg["feedback_type"])
                self.feedback_subs.append(
                    self.create_subscription(
                        typ,
                        cfg["feedback_topic"],
                        lambda m, s=side: self.feedback(s, m),
                        qos_profile_sensor_data,
                    )
                )
            if self.get_parameter("hardware_enabled").value:
                from deux_hand_msgs.msg import HandCtlMessage, MotorData

                self.Command, self.Motor = HandCtlMessage, MotorData
                self.command_pubs = {
                    s: self.create_publisher(HandCtlMessage, f"/hand/{s}/cmd", 10)
                    for s in ("left", "right")
                }
        self.action = ExclusiveAction(
            self, BridgeMotion, "/task/hand_motion", self.execute, self.validate
        )

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
            if self.config is None or not current(
                msg.header.stamp, self.now(), self.config["max_joint_age"]
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
                    self.raw[s][0].stamp, self.now(), self.config["max_joint_age"]
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
            self.get_logger().warning(str(error))

    def measured_hand(self):
        if self.config is None or not all(
            s in self.raw
            and current(self.raw[s][0].stamp, self.now(), self.config["max_joint_age"])
            for s in ("left", "right")
        ):
            raise ValueError("hand feedback stale/missing")
        values = {k: v for _, d in self.raw.values() for k, v in d.items()}
        return vector([values[n] for n in HAND_NAMES], 14)

    def validate(self, req):
        if req.scene_version != self.config_version:
            raise ValueError("installation config version mismatch")
        self.hardware_gate()
        if self.config is None or not (
            self.config.get("synthetic", False)
            or (
                self.config.get("calibration_confirmed") is True
                and self.config.get("command_ownership_confirmed") is True
            )
        ):
            raise ValueError("installation calibration/command ownership unconfirmed")
        if not self.command_pubs or self.mapping is None:
            raise ValueError("hand dispatch disabled/unconfigured")
        t, q = arrays(req.trajectory, HAND_NAMES)
        if (
            not self.now()
            < seconds(req.trajectory.header.stamp)
            < seconds(req.deadline)
        ):
            raise ValueError("invalid hand schedule")
        if np.max(abs(self.measured_hand() - q[0])) > self.config["start_tolerance"]:
            raise ValueError("hand start mismatch")
        for i, n in enumerate(HAND_NAMES):
            m = self.mapping[n.split("_")[0]]["joints"][n]
            if np.any(q[:, i] < m["min"]) or np.any(q[:, i] > m["max"]):
                raise ValueError("hand joint limit")
            if np.any(abs(np.diff(q[:, i]) / np.diff(t)) > m["velocity"]):
                raise ValueError("hand velocity limit")

    def command(self, positions):
        for side, pub in self.command_pubs.items():
            cfg = self.mapping[side]
            msg = self.Command()
            msg.header.stamp = self.get_clock().now().to_msg()
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

    async def execute(self, goal):
        req = goal.request
        self.validate(req)
        t, q = arrays(req.trajectory, HAND_NAMES)
        start = seconds(req.trajectory.header.stamp)
        wall = time.monotonic()
        previous = self.now()
        completed = False
        try:
            while True:
                now = self.now()
                elapsed = now - start
                if goal.is_cancel_requested:
                    raise InterruptedError("canceled")
                if (
                    now < previous
                    or now > seconds(req.deadline)
                    or time.monotonic() - wall > self.wall_timeout(t[-1] + 5)
                ):
                    raise TimeoutError("hand deadline/clock")
                self.hardware_gate()
                previous = now
                actual = self.measured_hand()
                desired = sample(t, q, max(0, elapsed))
                if np.max(abs(actual - desired)) > self.config["tracking_tolerance"]:
                    raise ValueError("hand tracking error")
                if elapsed >= 0:
                    self.command(desired)
                if (
                    elapsed >= t[-1]
                    and np.max(abs(actual - q[-1])) <= self.config["start_tolerance"]
                ):
                    completed = True
                    return BridgeMotion.Result(
                        success=True, reason="hand measured complete"
                    )
                goal.publish_feedback(
                    BridgeMotion.Feedback(
                        progress=float(np.clip(elapsed / t[-1], 0, 1)), status="hand"
                    )
                )
                await pause(self)
        finally:
            # A successful close must retain its commanded preload. Replacing
            # it with stalled measured angles unloads a compliant grasp.
            if not completed:
                try:
                    self.command(self.measured_hand())
                except ValueError:
                    self.get_logger().error(
                        "feedback lost: no guessed hold; driver watchdog required"
                    )


def main(args=None):
    run(HandBridgeNode, args)
