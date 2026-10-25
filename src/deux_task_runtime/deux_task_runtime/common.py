"""Action lifetime utilities; asynchronous waits keep subscriptions responsive."""

import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.task import Future
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Time, Duration
from scipy.spatial.transform import Rotation
from deux_task_msgs.msg import StateObservation, BarcodeTarget
from std_msgs.msg import String
from deux_task_core.model import JOINT_NAMES, vector
from deux_task_core.configuration import load_configuration
from deux_task_core.planning import scene_version

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def ros_time(value, cls=Time):
    ns = int(round(value * 1e9))
    return cls(sec=ns // 10**9, nanosec=ns % 10**9)


def pose_matrix(p):
    q = vector([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w], 4)
    if not np.isclose(np.linalg.norm(q), 1, atol=1e-3):
        raise ValueError("invalid orientation")
    t = np.eye(4)
    t[:3, 3] = vector([p.position.x, p.position.y, p.position.z], 3)
    t[:3, :3] = Rotation.from_quat(q).as_matrix()
    return t


def joint_array(msg, names=JOINT_NAMES):
    if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
        raise ValueError("malformed joints")
    mapping = dict(zip(msg.name, msg.position))
    return vector([mapping[n] for n in names], len(names))


def current(stamp, now, age):
    return 0 < seconds(stamp) <= now and now - seconds(stamp) <= age


async def pause(node, delay=0.02):
    future = Future()
    timer = node.create_timer(
        delay,
        lambda: future.set_result(True) if not future.done() else None,
        callback_group=node.group,
    )
    try:
        await future
    finally:
        node.destroy_timer(timer)


async def wait(node, future, timeout=3.0, goal=None):
    end = time.monotonic() + (node.wall_timeout(timeout) if hasattr(node, "wall_timeout") else timeout)
    while not future.done():
        if goal is not None and goal.is_cancel_requested:
            raise InterruptedError("canceled")
        if time.monotonic() > end:
            raise TimeoutError("action response timeout")
        await pause(node)
    return future.result()


async def cancel_children(node, handles):
    for handle in handles:
        if handle is not None and handle.accepted:
            try:
                response = await wait(node, handle.cancel_goal_async(), 2.0)
                if not response.goals_canceling:
                    # A terminal goal also legitimately has no canceling entry.
                    await wait(node, handle.get_result_async(), 2.0)
            except Exception as error:
                node.get_logger().error(f"cancel not confirmed: {error}")


class RuntimeNode(Node):
    def __init__(self, name, *, observe_state=False, observe_barcode=False,
                 observe_status=False, **kwargs):
        super().__init__(name, **kwargs)
        self.group = ReentrantCallbackGroup()
        self.declare_parameter("config_file", "", ParameterDescriptor(read_only=True))
        try:
            self.config, self.config_version = load_configuration(
                self.get_parameter("config_file").value
            )
            self.wall_timeout(1.0)
        except Exception as error:
            Node.destroy_node(self)
            raise ValueError(f"invalid installation config: {error}") from error
        self.get_logger().info(
            f"Loaded installation config version {self.config_version}"
        )
        self.observation = None
        self.latest_barcode_target = None
        self.robot_status = None
        # Each consumer opts into only the streams used by its role.
        self.subs = []
        if observe_barcode:
            self.subs.append(self.create_subscription(
                BarcodeTarget, "/task/barcode_target",
                lambda msg: setattr(self, "latest_barcode_target", msg),
                qos_profile_sensor_data,
            ))
        if observe_status:
            self.subs.append(self.create_subscription(
                String, "/state/status", self.on_status, LATCHED,
            ))
        if observe_state:
            self.subs.append(self.create_subscription(
                StateObservation, "/task/state_observation",
                self.on_observation, qos_profile_sensor_data,
            ))

    def on_status(self, msg):
        try:
            self.robot_status = json.loads(msg.data)
        except (ValueError, TypeError):
            self.robot_status = None

    def hardware_gate(self):
        c = self.config
        if c is None:
            raise ValueError("scene unavailable")
        if c.get("synthetic", False):
            return
        if (
            c.get("calibration_confirmed") is not True
            or c.get("command_ownership_confirmed") is not True
        ):
            raise ValueError("installation calibration/command ownership unconfirmed")
        if self.count_publishers("/arm/deux_arm_controller/joint_trajectory"):
            raise ValueError(
                "competing direct arm command publisher; isolate teleop/state launch"
            )
        if c.get("require_state_status", True):
            if (
                self.robot_status is None
                or self.robot_status.get("teleop") != "off"
                or self.robot_status.get("mode") not in c["allowed_state_modes"]
            ):
                raise ValueError("state mode/teleop not admitted")
            if not self.count_publishers("/state/status"):
                raise ValueError("state status publisher lost")

    def wall_timeout(self, duration):
        # ROS deadlines stay in simulation time; retain a bounded wall watchdog
        # for slow software rendering or paused simulation.
        scale = float(self.config.get('simulation_wall_timeout_scale', 1.))
        if scale != 1. and not (self.config.get('synthetic', False)
                and self.get_parameter('use_sim_time').value):
            raise ValueError('wall timeout scaling requires synthetic scene and use_sim_time')
        return duration * scale

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def check_model_version(self):
        if scene_version(self.config) != self.config_version:
            raise ValueError(
                "URDF changed since startup; restart with consistent configuration"
            )

    def on_observation(self, msg):
        self.observation = msg

    def check_target_rotation(self, actual, planned):
        rotation = pose_matrix(actual)[:3, :3] @ pose_matrix(planned)[:3, :3].T
        if Rotation.from_matrix(rotation).magnitude() > self.config.get(
            "max_target_angle_rad", 0.15
        ):
            raise ValueError("target rotated; replan")

    def measured(self, obs=None):
        obs = self.observation if obs is None else obs
        if self.config is None or obs is None:
            raise ValueError("scene/observation unavailable")
        now = self.now()
        c = self.config
        if not current(obs.header.stamp, now, c["max_observation_age"]):
            raise ValueError("stale/future observation")
        if not obs.joints_valid or not current(
            obs.joints.header.stamp, now, c["max_joint_age"]
        ):
            raise ValueError("measured joints unavailable/stale")
        return joint_array(obs.joints)

    def target(self, identity, obs=None, full=False):
        obs = self.observation if obs is None else obs
        if self.config is None or obs is None or not obs.objects_valid:
            raise ValueError("objects unavailable")
        if (
            obs.objects.header.frame_id != self.config["base_frame"]
            or obs.objects.model_id != self.config["model_id"]
        ):
            raise ValueError("object frame/model origin mismatch")
        if not current(
            obs.objects.header.stamp, self.now(), self.config["max_object_age"]
        ):
            raise ValueError("object stale/future")
        obj = next((d for d in obs.objects.detections if d.track_id == identity), None)
        if obj is None or (full and not obj.full_orientation):
            raise ValueError("required object pose unavailable")
        return obj


class ExclusiveAction:
    def __init__(self, node, action, name, execute, validate=lambda request: None):
        self.node, self.action, self.execute, self.validate = (
            node,
            action,
            execute,
            validate,
        )
        self.busy = False
        self.server = ActionServer(
            node,
            action,
            name,
            execute_callback=self.run,
            goal_callback=self.accept,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=node.group,
        )

    def accept(self, request):
        if self.busy:
            return GoalResponse.REJECT
        try:
            self.validate(request)
        except Exception as e:
            self.node.get_logger().warning(str(e))
            return GoalResponse.REJECT
        self.busy = True
        return GoalResponse.ACCEPT

    async def run(self, goal):
        result = self.action.Result()
        try:
            result = await self.execute(goal)
            if goal.is_cancel_requested:
                goal.canceled()
                result.success = False
                result.reason = "canceled"
            elif result.success:
                goal.succeed()
            else:
                goal.abort()
        except Exception as error:
            result.success = False
            result.reason = f"{type(error).__name__}: {error}"
            if goal.is_cancel_requested:
                goal.canceled()
            else:
                goal.abort()
        finally:
            self.busy = False
        return result


def run(cls, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = cls()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
