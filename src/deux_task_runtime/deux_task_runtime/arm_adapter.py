"""Arm I/O adapter owned by ExecutionNode. DEUX driver action stays unchanged."""
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from deux_task_core.model import ARM_NAMES
from .common import current, joint_array, cancel_children


class ArmAdapter:
    def __init__(self, node):
        self.node = node
        self.state = None
        self.pending = None
        self.result = None
        self.sub = node.create_subscription(JointState, "/arm/joint_states", self.receive,
                                            qos_profile_sensor_data)
        self.client = ActionClient(node, FollowJointTrajectory,
            node.get_parameter("driver_action").value, callback_group=node.group)

    def receive(self, msg):
        self.state = msg

    def measured_arm(self):
        if self.state is None or not current(self.state.header.stamp, self.node.now(),
                                             self.node.config["max_joint_age"]):
            raise ValueError("arm feedback stale/missing")
        return joint_array(self.state, ARM_NAMES)

    def validate(self):
        if not self.node.get_parameter("hardware_enabled").value:
            raise ValueError("arm dispatch disabled")
        if self.pending is not None:
            if not self.pending.done():
                raise ValueError("previous arm acceptance pending")
            previous = self.pending.result()
            if previous.accepted and not self.result_for(previous).done():
                raise ValueError("previous arm execution/cancellation pending")
        if not self.client.server_is_ready():
            raise ValueError("arm driver unavailable")

    def send(self, trajectory):
        self.result = None
        self.pending = self.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
        return self.pending

    def result_for(self, handle):
        if self.result is None:
            self.result = handle.get_result_async()
        return self.result

    @staticmethod
    def check_result(future):
        if not future.done():
            return False
        value = future.result()
        if (value.status != GoalStatus.STATUS_SUCCEEDED or
                value.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL):
            raise ValueError("arm failed: " + value.result.error_string)
        return True

    async def cancel(self, pending, handle):
        if handle is not None:
            await cancel_children(self.node, [handle])
        elif pending is not None:
            # A late acceptance still needs cancellation after an execution timeout.
            def cancel_late(future):
                try:
                    accepted = future.result()
                    if accepted.accepted:
                        accepted.cancel_goal_async()
                except Exception as error:
                    self.node.get_logger().error(f"late arm cancellation failed: {error}")
            if pending.done():
                accepted = pending.result()
                await cancel_children(self.node, [accepted])
            else:
                pending.add_done_callback(cancel_late)
