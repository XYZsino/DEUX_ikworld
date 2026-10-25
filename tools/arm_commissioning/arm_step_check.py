"""Read-only by default. Explicit reviewed plan required for one small arm-only step.

No hand publisher, no controller switching, no gain or e-stop reset commands.
This is a commissioning path, not the task/robot-base calibrated motion pipeline.
"""

import argparse
from collections import deque
import json
import signal
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState, JointTolerance
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from builtin_interfaces.msg import Duration
from arm_step_core import (
    ARM_NAMES,
    joint_vector,
    make_plan,
    fingerprint,
    cancellation_complete,
    status_problems,
    motion_guard,
    TOLERANCES,
)

BASE = "/arm/deux_arm_controller"
ACTION = BASE + "/follow_joint_trajectory"


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ArmStepCheck(Node):
    def __init__(self, with_gravity=False):
        super().__init__("deux_arm_step_check")
        self.state = {"controller_active": False}
        self.with_gravity = with_gravity
        self.gravity_state = None
        self.unexpected_trajectories = 0
        # Publisher presence is the existing gravity node's reservation signal.
        # This endpoint NEVER publishes a trajectory; actions carry the command.
        self.reservation = (
            self.create_publisher(JointTrajectory, BASE + "/joint_trajectory", 10)
            if with_gravity
            else None
        )
        self.joints = None
        self.controller = None
        self.history = deque(maxlen=150)
        self.interrupted = False
        self.handles = []
        self.send_future = None
        self.goal_statuses = []
        self.trace = []
        self.subs = [
            self.create_subscription(
                JointState, "/arm/joint_states", self.on_joints, qos_profile_sensor_data
            ),
            self.create_subscription(
                JointTrajectoryControllerState,
                BASE + "/controller_state",
                self.on_controller,
                qos_profile_sensor_data,
            ),
            self.create_subscription(Bool, "/deux/estop", self.on_estop, 10),
            self.create_subscription(
                JointTrajectory, BASE + "/joint_trajectory", self.on_trajectory, 10
            ),
            self.create_subscription(
                String, "/deux/stiffness/state", self.on_stiffness, 10
            ),
            self.create_subscription(
                GoalStatusArray,
                ACTION + "/_action/status",
                self.on_status,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            ),
        ]
        self.controller_client = self.create_client(
            ListControllers, "/arm/controller_manager/list_controllers"
        )
        self.description_client = self.create_client(
            GetParameters, "/arm/robot_state_publisher/get_parameters"
        )

    def on_joints(self, msg):
        self.joints = msg
        self.state["joints"] = {
            "received": time.monotonic(),
            "stamp": seconds(msg.header.stamp),
        }
        try:
            q = joint_vector(msg)
            self.history.append((time.monotonic(), q))
            if len(self.trace) < 10000:
                self.trace.append(
                    {"stamp": seconds(msg.header.stamp), "position": q.tolist()}
                )
        except (ValueError, KeyError):
            self.state.pop("joints", None)

    def on_controller(self, msg):
        self.controller = msg
        self.state["controller"] = {
            "received": time.monotonic(),
            "stamp": seconds(msg.header.stamp),
        }

    def on_stiffness(self, msg):
        try:
            value = json.loads(msg.data)
            self.gravity_state = value
            if type(value["estop"]) is not bool or not np.isfinite(float(value["t"])):
                raise ValueError("invalid stiffness state")
            self.state["stiffness"] = {
                "received": time.monotonic(),
                "stamp": float(value["t"]),
            }
            self.state["stiffness_applied"] = value["applied"]
            self.state["stiffness_mode"] = value["mode"]
            if value["estop"]:
                self.state["estop_seen"] = True
        except (ValueError, TypeError, KeyError):
            self.state.pop("stiffness", None)

    def on_estop(self, msg):
        if msg.data:
            self.state["estop_seen"] = True

    def on_status(self, msg):
        self.goal_statuses = msg.status_list
        self.refresh_goal_status()

    def refresh_goal_status(self):
        own_ids = {bytes(h.goal_id.uuid) for h in self.handles}
        self.state["active_goals"] = [
            list(s.goal_info.goal_id.uuid)
            for s in self.goal_statuses
            if s.status in (1, 2, 3) and bytes(s.goal_info.goal_id.uuid) not in own_ids
        ]

    def wait(self, future, timeout):
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end:
                raise TimeoutError("ROS response timeout")
            rclpy.spin_once(self, timeout_sec=0.02)
        return future.result()

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def refresh_controller(self):
        if not self.controller_client.wait_for_service(timeout_sec=2):
            raise RuntimeError("controller manager unavailable")
        response = self.wait(
            self.controller_client.call_async(ListControllers.Request()), 2
        )
        self.state["controller_active"] = any(
            c.name == "deux_arm_controller"
            and c.state == "active"
            and c.type == "joint_trajectory_controller/JointTrajectoryController"
            for c in response.controller
        )

    def urdf(self):
        if not self.description_client.wait_for_service(timeout_sec=2):
            raise RuntimeError("robot_description unavailable")
        return (
            self.wait(
                self.description_client.call_async(
                    GetParameters.Request(names=["robot_description"])
                ),
                2,
            )
            .values[0]
            .string_value
        )

    def on_trajectory(self, msg):
        self.unexpected_trajectories += 1

    def ownership_problems(self):
        own = self.get_namespace().rstrip("/") + "/" + self.get_name()
        pubs = [
            i.node_namespace.rstrip("/") + "/" + i.node_name
            for i in self.get_publishers_info_by_topic(BASE + "/joint_trajectory")
        ]
        others = [p for p in pubs if p != own]
        issues = []
        for topic in ("/arm/joint_states", BASE + "/controller_state"):
            if len(self.get_publishers_info_by_topic(topic)) != 1:
                issues.append("expected one feedback publisher: " + topic)
        if self.with_gravity:
            # Narrow cooperation with this specific stack, never an allow-any override.
            if others != ["/deux_gravity"]:
                issues.append(
                    "expected exactly one gravity publisher and no other command source"
                )
            if own not in pubs:
                issues.append("command reservation endpoint missing")
            v = self.gravity_state or {}
            try:
                valid = (
                    v["mode"] == "gravity"
                    and v["applied"] == "setpoint"
                    and v["estop"] is False
                    and v["ramping"] is False
                    and v["probe"] is False
                    and all(
                        v["contact"][side]["active"] is False
                        for side in ("left", "right")
                    )
                    and all(
                        len(v["target"][side]["kp"]) == 7
                        and all(
                            np.isfinite(k) and k > 0 for k in v["target"][side]["kp"]
                        )
                        for side in ("left", "right")
                    )
                )
            except (KeyError, TypeError):
                valid = False
            if not valid:
                issues.append("gravity not in stable position-holding state")
            self.state["command_publishers"] = [
                p for p in others if p != "/deux_gravity"
            ]
        else:
            self.state["command_publishers"] = others
        if self.unexpected_trajectories:
            issues.append("unexpected direct trajectory received")
        return issues

    def problems(self):
        self.refresh_goal_status()
        ownership = self.ownership_problems()
        errors = status_problems(
            self.state, time.monotonic(), self.get_clock().now().nanoseconds * 1e-9
        )
        errors.extend(ownership)
        try:
            actual = joint_vector(self.joints)
            ctrl = self.controller
            if list(ctrl.joint_names) != list(ARM_NAMES):
                errors.append("controller joint order differs")
            elif (
                len(ctrl.reference.positions) != 14
                or not np.isfinite(ctrl.reference.positions).all()
            ):
                errors.append("controller reference invalid")
            elif np.max(
                np.abs(actual - np.array(ctrl.reference.positions))
            ) > np.deg2rad(TOLERANCES["start_reference_error_deg"]):
                errors.append(
                    "controller target differs from measured posture by >0.5deg"
                )
        except (AttributeError, ValueError, KeyError):
            errors.append("invalid controller/joint feedback")
        return errors

    def stationary(self):
        now = time.monotonic()
        recent = [(t, q) for t, q in self.history if now - t <= 0.6]
        return bool(
            len(recent) >= 5
            and recent[-1][0] - recent[0][0] >= 0.4
            and np.max(np.ptp(np.array([q for _, q in recent]), axis=0)) < 0.003
        )

    def confirm_stopped(self, timeout=3.0):
        start = time.monotonic()
        self.history.clear()
        while time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.02)
            stream = self.state.get("joints")
            if (
                stream
                and 0 <= time.monotonic() - stream["received"] <= 0.2
                and 0
                <= self.get_clock().now().nanoseconds * 1e-9 - stream["stamp"]
                <= 0.2
                and len(self.get_publishers_info_by_topic("/arm/joint_states")) == 1
                and self.stationary()
            ):
                return
        raise RuntimeError("STOP UNCONFIRMED: no fresh stationary measured feedback")

    def execute(self, report, approved):
        if self.problems():
            raise RuntimeError("; ".join(self.problems()))
        if not self.stationary():
            raise RuntimeError("arm not stationary")
        plan = approved["plan"]
        if approved.get("tolerances") != TOLERANCES:
            raise RuntimeError("tolerances changed since preview")
        if self.with_gravity != approved.get("with_gravity", False):
            raise RuntimeError("ownership mode changed since preview")
        if self.state.get("stiffness_mode") != approved.get("stiffness_mode"):
            raise RuntimeError("stiffness mode changed since reviewed preview")
        if approved.get("blockers"):
            raise RuntimeError(
                "approved preview contains blockers; generate a clean preview first"
            )
        age = time.time() - approved["created_unix_s"]
        if not 0 <= age <= TOLERANCES["preview_max_age_s"]:
            raise RuntimeError("reviewed plan older than 60s; preview again")
        if fingerprint(plan) != approved.get("plan_sha256"):
            raise RuntimeError("plan hash mismatch")
        if (
            np.max(np.abs(joint_vector(self.joints) - plan["start_rad"]))
            > TOLERANCES["preview_start_drift_rad"]
        ):
            raise RuntimeError("posture moved since reviewed preview")
        verified = make_plan(
            plan["start_rad"],
            plan["joint"],
            plan["delta_deg"],
            plan["duration_s"],
            self.urdf(),
        )
        if fingerprint(verified) != approved["plan_sha256"]:
            raise RuntimeError("URDF/trajectory changed since review")
        report.update(
            plan=plan,
            plan_sha256=approved["plan_sha256"],
            reviewed_plan_created_unix_s=approved["created_unix_s"],
        )
        client = ActionClient(self, FollowJointTrajectory, ACTION)
        if not client.wait_for_server(timeout_sec=2):
            raise RuntimeError("driver Action unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM_NAMES)
        # A future start leaves time to receive acceptance. Position-only linear
        # interpolation between dense quintic samples; no final velocity command.
        start = self.get_clock().now().nanoseconds * 1e-9 + 0.5
        ns = int(start * 1e9)
        goal.trajectory.header.stamp.sec, goal.trajectory.header.stamp.nanosec = divmod(
            ns, 10**9
        )
        for t, q in zip(plan["times_s"], plan["positions_rad"]):
            sec, nano = divmod(int(round(t * 1e9)), 10**9)
            goal.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=q, time_from_start=Duration(sec=sec, nanosec=nano)
                )
            )
        goal.path_tolerance = [
            JointTolerance(
                name=n, position=float(np.deg2rad(TOLERANCES["driver_path_error_deg"]))
            )
            for n in ARM_NAMES
        ]
        goal.goal_tolerance = [
            JointTolerance(
                name=n,
                position=float(np.deg2rad(TOLERANCES["goal_position_error_deg"])),
            )
            for n in ARM_NAMES
        ]
        goal.goal_time_tolerance = Duration(
            sec=int(TOLERANCES["goal_time_allowance_s"])
        )
        self.refresh_controller()
        if self.problems() or self.interrupted:
            raise RuntimeError("pre-dispatch gate changed or interrupted")
        handle = None
        result = None
        result_future = None
        sent_at = time.monotonic()
        try:
            self.send_future = client.send_goal_async(goal)
            report["goal_sent"] = True
            handle = self.wait(self.send_future, 0.3)
            if not handle.accepted:
                raise RuntimeError("driver rejected goal")
            self.handles.append(handle)
            report["goal_uuid"] = [int(v) for v in handle.goal_id.uuid]
            result_future = handle.get_result_async()
            path_array = np.asarray(plan["positions_rad"])
            while not result_future.done():
                if self.interrupted:
                    raise InterruptedError("operator interrupted; cancel requested")
                if self.get_clock().now().nanoseconds * 1e-9 < start - 0.7:
                    raise RuntimeError("clock moved backwards")
                if time.monotonic() - sent_at > plan["duration_s"] + 4:
                    raise TimeoutError("motion deadline")
                # During motion, desired position intentionally differs from actual;
                # guard freshness/ownership and explicit planned following error.
                ownership = self.ownership_problems()
                self.refresh_goal_status()
                issues = status_problems(
                    self.state,
                    time.monotonic(),
                    self.get_clock().now().nanoseconds * 1e-9,
                )
                issues.extend(ownership)
                if self.state.get("stiffness_mode") != approved.get("stiffness_mode"):
                    issues.append("stiffness mode changed during move")
                if issues:
                    raise RuntimeError("; ".join(issues))
                elapsed = max(0, self.get_clock().now().nanoseconds * 1e-9 - start)
                expected = np.array(
                    [
                        np.interp(elapsed, plan["times_s"], path_array[:, i])
                        for i in range(14)
                    ]
                )
                motion_guard(joint_vector(self.joints), expected)
                rclpy.spin_once(self, timeout_sec=0.02)
            result = result_future.result()
            report["driver_status"] = result.status
            report["driver_error_code"] = result.result.error_code
            report["driver_error_string"] = result.result.error_string
            report["terminal_status"] = int(result.status)
            if result.status != 4 or result.result.error_code != 0:
                raise RuntimeError("driver failed: " + result.result.error_string)
            self.confirm_stopped()
            final_expected = np.array(plan["target_rad"])
            if np.max(np.abs(joint_vector(self.joints) - final_expected)) > np.deg2rad(
                TOLERANCES["goal_position_error_deg"]
            ):
                raise RuntimeError("final measured goal mismatch")
            report["stop_confirmed"] = True
        except Exception as error:
            report["primary_error"] = f"{type(error).__name__}: {error}"
            raise
        finally:
            try:
                if report.get("goal_sent") and not report.get("stop_confirmed"):
                    try:
                        if handle is None:
                            handle = self.wait(self.send_future, 3.0)
                            if handle.accepted:
                                self.handles.append(handle)
                        outcome = {}
                        report["cleanup"] = outcome
                        terminal = cancellation_complete(
                            handle, self.wait, result_future, outcome
                        )
                        if terminal is not None:
                            report["terminal_status"] = int(terminal.status)
                            report["terminal_error_code"] = int(
                                terminal.result.error_code
                            )
                            report["terminal_error_string"] = (
                                terminal.result.error_string
                            )
                            report["canceled_after_failure"] = terminal.status == 5
                    except Exception as cleanup_error:
                        report["cleanup_error"] = (
                            f"{type(cleanup_error).__name__}: {cleanup_error}"
                        )
                    # A physical stop and an Action terminal result are different facts.
                    try:
                        self.confirm_stopped()
                        report["stop_confirmed"] = True
                    except Exception as stop_error:
                        report["stop_confirmed"] = False
                        report["stop_error"] = (
                            f"{type(stop_error).__name__}: {stop_error}"
                        )
            finally:
                client.destroy()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", default="arm_r_joint7")
    parser.add_argument("--delta-deg", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--with-gravity",
        action="store_true",
        help="Reserve trajectory ownership while verified deux_gravity supplies compensation",
    )
    parser.add_argument(
        "--execute-reviewed-plan",
        help="Explicit opt-in; requires recent blocker-free preview JSON",
    )
    opts = parser.parse_args(args)
    rclpy.init(args=[])
    node = ArmStepCheck(with_gravity=opts.with_gravity)
    report = {
        "mode": "execute" if opts.execute_reviewed_plan else "preview",
        "goal_sent": False,
        "hand_commands": False,
        "with_gravity": opts.with_gravity,
        "created_unix_s": time.time(),
        "tolerances": dict(TOLERANCES),
    }
    previous = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    for sig in previous:
        signal.signal(sig, lambda *_: setattr(node, "interrupted", True))
    code = 0
    try:
        node.spin_for(1.2)
        node.refresh_controller()
        urdf = node.urdf()
        # Service discovery/calls can leave queued telemetry. Drain it and require
        # the same freshness and stationarity gates; never dispatch on timeout.
        ready_deadline = time.monotonic() + 4.0
        while time.monotonic() < ready_deadline:
            node.spin_for(0.1)
            if node.stationary() and not node.problems():
                break
        plan = make_plan(
            joint_vector(node.joints), opts.joint, opts.delta_deg, opts.duration, urdf
        )
        motion_guard(plan["start_rad"], plan["start_rad"])
        report.update(
            plan=plan,
            plan_sha256=fingerprint(plan),
            stiffness_mode=node.state.get("stiffness_mode"),
            blockers=node.problems(),
            stationary=node.stationary(),
        )
        if not report["stationary"]:
            report["blockers"].append("measured posture not stationary")
        if node.interrupted:
            raise InterruptedError("operator interrupted before dispatch")
        if opts.execute_reviewed_plan:
            node.execute(
                report, json.loads(Path(opts.execute_reviewed_plan).read_text())
            )
        report["result"] = "completed" if report.get("goal_sent") else "preview_only"
    except Exception as error:
        report["result"] = "failed"
        report["error"] = f"{type(error).__name__}: {error}"
        code = 1
    finally:
        report["trace"] = node.trace
        Path(opts.output).parent.mkdir(parents=True, exist_ok=True)
        Path(opts.output).write_text(json.dumps(report, indent=2, allow_nan=False))
        print(
            json.dumps(
                {k: v for k, v in report.items() if k not in ("plan", "trace")},
                indent=2,
            )
        )
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        node.destroy_node()
        rclpy.shutdown()
    if code:
        raise SystemExit(code)


if __name__ == "__main__":
    main()
