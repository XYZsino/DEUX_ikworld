import time
import numpy as np
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from deux_task_msgs.action import BridgeMotion
from deux_task_core.model import ARM_NAMES, RobotModel
from deux_task_runtime.common import (
    RuntimeNode,
    ExclusiveAction,
    current,
    joint_array,
    seconds,
    pause,
    wait,
    cancel_children,
    run,
)
from deux_task_runtime.trajectory import arrays, sample, validate_limits


class ArmBridgeNode(RuntimeNode):
    def __init__(self, **kwargs):
        super().__init__("arm_bridge_node", observe_status=True, **kwargs)
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter(
            "driver_action", "/arm/deux_arm_controller/follow_joint_trajectory"
        )
        self.state = None
        self.sub = self.create_subscription(
            JointState,
            "/arm/joint_states",
            lambda m: setattr(self, "state", m),
            qos_profile_sensor_data,
        )
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            self.get_parameter("driver_action").value,
            callback_group=self.group,
        )
        self.action = ExclusiveAction(
            self, BridgeMotion, "/task/arm_motion", self.execute, self.validate
        )

    def measured_arm(self):
        if (
            self.config is None
            or self.state is None
            or not current(
                self.state.header.stamp, self.now(), self.config["max_joint_age"]
            )
        ):
            raise ValueError("arm feedback stale/missing")
        return joint_array(self.state, ARM_NAMES)

    def validate(self, req):
        self.check_model_version()
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
        if not self.get_parameter("hardware_enabled").value:
            raise ValueError("arm dispatch disabled")
        t, q = validate_limits(
            req.trajectory, ARM_NAMES, RobotModel(self.config["robot"])
        )
        if (
            not self.now()
            < seconds(req.trajectory.header.stamp)
            < seconds(req.deadline)
        ):
            raise ValueError("invalid scheduled time")
        if np.max(abs(self.measured_arm() - q[0])) > self.config["start_tolerance"]:
            raise ValueError("arm start mismatch")
        if not self.client.server_is_ready():
            raise ValueError("arm driver unavailable")

    async def execute(self, goal):
        req = goal.request
        self.validate(req)
        t, q = arrays(req.trajectory, ARM_NAMES)
        send = self.client.send_goal_async(
            FollowJointTrajectory.Goal(trajectory=req.trajectory)
        )
        handle = None
        try:
            handle = await wait(
                self,
                send,
                max(0.01, seconds(req.trajectory.header.stamp) - self.now() - 0.1),
                goal,
            )
            if not handle.accepted:
                raise ValueError("arm driver rejected")
            result = handle.get_result_async()
            start = seconds(req.trajectory.header.stamp)
            wall = time.monotonic()
            prev = self.now()
            while not result.done():
                now = self.now()
                if goal.is_cancel_requested:
                    raise InterruptedError("canceled")
                if (
                    now < prev
                    or now > seconds(req.deadline)
                    or time.monotonic() - wall > self.wall_timeout(t[-1] + 5)
                ):
                    raise TimeoutError("arm deadline/clock")
                prev = now
                self.hardware_gate()
                if (
                    np.max(abs(self.measured_arm() - sample(t, q, max(0, now - start))))
                    > self.config["tracking_tolerance"]
                ):
                    raise ValueError("arm tracking error")
                goal.publish_feedback(
                    BridgeMotion.Feedback(
                        progress=float(np.clip((now - start) / t[-1], 0, 1)),
                        status="arm",
                    )
                )
                await pause(self)
            value = result.result()
            if (
                value.status != GoalStatus.STATUS_SUCCEEDED
                or value.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
            ):
                raise ValueError("arm failed: " + value.result.error_string)
            if (
                np.max(abs(self.measured_arm() - q[-1]))
                > self.config["start_tolerance"]
            ):
                raise ValueError("arm final mismatch")
            return BridgeMotion.Result(success=True, reason="arm measured complete")
        finally:
            if handle is not None:
                await cancel_children(self, [handle])
            elif not send.done():
                send.add_done_callback(
                    lambda f: (
                        f.result().cancel_goal_async() if f.result().accepted else None
                    )
                )


def main(args=None):
    run(ArmBridgeNode, args)
