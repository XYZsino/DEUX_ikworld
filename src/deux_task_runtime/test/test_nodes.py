import json
import time
from copy import deepcopy
from pathlib import Path
import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.msg import (
    StateObservation,
    ObjectDetection,
    ScanSession,
    BarcodeTarget,
    BarcodeRead,
)
from deux_task_msgs.action import PlanMotion, ExecuteMotion, BridgeMotion
from deux_task_core.synthetic import fixture
from deux_task_core.model import JOINT_NAMES, HAND_NAMES, ARM_NAMES
from deux_task_core.planning import scene_version
from deux_task_runtime.common import LATCHED, ros_time, seconds
from deux_task_runtime.motion_planner_node import MotionPlannerNode
from deux_task_runtime.execution_node import ExecutionNode
from deux_task_runtime.task_node import TaskNode
from deux_task_adapters.arm_bridge_node import ArmBridgeNode
from deux_task_adapters.hand_bridge_node import HandBridgeNode
from deux_task_perception.barcode_bridge_node import BarcodeBridgeNode


def params(**kwargs):
    return [Parameter(k, value=v) for k, v in kwargs.items()]


class Harness:
    next_domain = 101

    def __init__(self, config):
        rclpy.init(domain_id=Harness.next_domain)
        Harness.next_domain += 1
        self.ex = SingleThreadedExecutor()
        self.nodes = []
        self.config = config
        self.source = rclpy.create_node("synthetic_source")
        self.add(self.source)
        self.config_path = (
            Path(config["robot"]["urdf_path"]).parent / "installation.yaml"
        )
        self.config_path.write_text(yaml.safe_dump(config))
        self.pub = self.source.create_publisher(
            StateObservation, "/task/state_observation", qos_profile_sensor_data
        )
        self.q = np.zeros(28)
        self.sequence = 0
        self.publish_enabled = True
        self.object_z = 0.0
        self.separated = False
        self.timer = self.source.create_timer(0.03, self.publish)

    def add(self, node):
        self.nodes.append(node)
        self.ex.add_node(node)
        return node

    def publish(self):
        if not self.publish_enabled:
            return
        now = self.source.get_clock().now().to_msg()
        obs = StateObservation()
        obs.header.stamp = now
        obs.header.frame_id = "synthetic_optical"
        obs.sequence = self.sequence
        obs.joints_valid = True
        obs.joints.header.stamp = now
        obs.joints.name = list(JOINT_NAMES)
        obs.joints.position = self.q.tolist()
        obs.objects_valid = True
        obs.objects.header.stamp = now
        obs.objects.header.frame_id = "base_link"
        obs.objects.sequence = self.sequence
        obs.objects.model_id = "synthetic-object"
        obs.objects.source = "synthetic"
        obj = ObjectDetection(track_id="bag", confidence=1.0, full_orientation=True)
        obj.pose.pose.orientation.w = 1.0
        obj.pose.pose.position.x = 0.48
        obj.pose.pose.position.y = -0.4
        obj.pose.pose.position.z = self.object_z
        obs.right_ee_valid = True
        obs.base_from_right_ee.header.stamp = now
        obs.base_from_right_ee.header.frame_id = "base_link"
        obs.base_from_right_ee.transform.rotation.w = 1.0
        obs.base_from_right_ee.transform.translation.x = 1.0 if self.separated else 0.48
        obs.base_from_right_ee.transform.translation.y = -0.4
        obs.objects.detections = [obj]
        self.pub.publish(obs)
        self.observation = obs
        self.sequence += 1

    def spin(self, predicate, timeout=6):
        end = time.monotonic() + timeout
        while time.monotonic() < end and not predicate():
            self.ex.spin_once(timeout_sec=0.02)
        assert predicate(), "ROS test timeout"

    def future(self, future, timeout=6):
        self.spin(future.done, timeout)
        return future.result()

    def close(self):
        self.ex.shutdown()
        for node in reversed(self.nodes):
            node.destroy_node()
        rclpy.shutdown()


def test_barcode_session_bounds(tmp_path):
    h = Harness(fixture(tmp_path))
    h.publish_enabled = False
    barcode = h.add(BarcodeBridgeNode())
    reads = []
    subs = [
        h.source.create_subscription(
            BarcodeRead, "/task/barcode_read", reads.append, 10
        ),
    ]
    raw = h.source.create_publisher(String, "/barcode/raw", 10)
    session = h.source.create_publisher(ScanSession, "/task/scan_session", LATCHED)
    try:
        h.spin(
            lambda: raw.get_subscription_count() > 0
            and session.get_subscription_count() > 0
        )
        raw.publish(String(data="before"))
        h.spin(lambda: barcode.session is None)
        now = h.source.get_clock().now().nanoseconds * 1e-9
        s = ScanSession(
            session_id="scan1",
            track_id="bag",
            active=True,
            expires_at=ros_time(now + 2),
        )
        s.header.stamp = ros_time(now)
        session.publish(s)
        h.spin(lambda: barcode.session is not None)
        raw.publish(String(data="12345"))
        h.spin(lambda: bool(reads))
        assert (
            reads[-1].session_id == "scan1"
            and reads[-1].track_id == "bag"
            and not reads[-1].simulated
        )
        count = len(reads)
        raw.publish(String(data="12345"))
        for _ in range(10):
            h.ex.spin_once(timeout_sec=0.01)
        assert len(reads) == count
        s.active = False
        session.publish(s)
        h.spin(lambda: not barcode.session.active)
        raw.publish(String(data="after"))
        for _ in range(10):
            h.ex.spin_once(timeout_sec=0.01)
        assert len(reads) == count
    finally:
        h.close()


def test_process_planner_and_dry_execution_rejects_stale_duplicate_and_changed_scene(
    tmp_path,
):
    h = Harness(fixture(tmp_path))
    planner = h.add(
        MotionPlannerNode(parameter_overrides=params(config_file=str(h.config_path)))
    )
    executor = h.add(
        ExecutionNode(parameter_overrides=params(config_file=str(h.config_path)))
    )
    client = ActionClient(h.source, PlanMotion, "/task/plan_motion")
    execute = ActionClient(h.source, ExecuteMotion, "/task/execute_motion")
    try:
        h.spin(
            lambda: planner.observation is not None
            and executor.observation is not None
            and client.server_is_ready()
        )
        request = PlanMotion.Goal(
            stage="Close",
            target_id="bag",
            scene_version=scene_version(h.config),
            observation=h.observation,
        )
        handle = h.future(client.send_goal_async(request))
        assert handle.accepted
        result = h.future(handle.get_result_async(), 15).result
        assert result.success, result.reason
        plan = result.plan
        rotated = deepcopy(plan.target_pose)
        rotated.orientation.z = np.sin(0.4)
        rotated.orientation.w = np.cos(0.4)
        with pytest.raises(ValueError, match="rotated"):
            executor.check_target_rotation(plan.target_pose, rotated)
        assert len(plan.trajectory.points) > 2
        h.spin(execute.server_is_ready)
        accepted = h.future(execute.send_goal_async(ExecuteMotion.Goal(plan=plan)))
        assert accepted.accepted
        done = h.future(accepted.get_result_async()).result
        assert done.success and done.reason == "dry_run_only", done.reason
        repeated = h.future(execute.send_goal_async(ExecuteMotion.Goal(plan=plan)))
        assert not repeated.accepted
        plan.plan_id = "expired"
        plan.valid_until = ros_time(1.0)
        expired = h.future(execute.send_goal_async(ExecuteMotion.Goal(plan=plan)))
        assert not expired.accepted
        request.scene_version = "wrong"
        assert not h.future(client.send_goal_async(request)).accepted
        request.scene_version = scene_version(h.config)
        request.observation.joints_valid = False
        assert not h.future(client.send_goal_async(request)).accepted
    finally:
        h.close()


def test_hand_feedback_mapping_preserves_stamp_and_dispatch_disabled(tmp_path):
    c = fixture(tmp_path)
    mapping = dict(calibrated=True, position_mode_confirmed=True)
    for side in ("left", "right"):
        names = [n for n in HAND_NAMES if n.startswith(side)]
        mapping[side] = dict(
            feedback_topic=f"/synthetic/{side}/feedback",
            feedback_type="sensor_msgs/msg/JointState",
            mode=0,
            action_state=0,
            action_state_save=0,
            joints={
                n: dict(
                    id=i,
                    feedback_name=str(i),
                    sign=-1,
                    offset=0.2,
                    min=-1.0,
                    max=1.0,
                    velocity=1.0,
                    goal_current=0.0,
                )
                for i, n in enumerate(names)
            },
        )
    path = tmp_path / "mapping.yaml"
    path.write_text(yaml.safe_dump(mapping))
    h = Harness(c)
    hand = h.add(
        HandBridgeNode(
            parameter_overrides=params(
                config_file=str(h.config_path), mapping_file=str(path)
            )
        )
    )
    arm = h.add(
        ArmBridgeNode(parameter_overrides=params(config_file=str(h.config_path)))
    )
    received = []
    sub = h.source.create_subscription(
        JointState, "/task/hand_state", received.append, qos_profile_sensor_data
    )
    pubs = {
        s: h.source.create_publisher(
            JointState, f"/synthetic/{s}/feedback", qos_profile_sensor_data
        )
        for s in ("left", "right")
    }
    try:
        h.spin(
            lambda: hand.config is not None
            and all(p.get_subscription_count() for p in pubs.values())
        )
        stamp = h.source.get_clock().now().to_msg()
        for pub in pubs.values():
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [str(i) for i in range(7)]
            msg.position = [0.1] * 7
            pub.publish(msg)
        h.spin(lambda: bool(received))
        np.testing.assert_allclose(received[-1].position, [0.1] * 14)
        assert received[-1].header.stamp == stamp
        req = BridgeMotion.Goal(scene_version=hand.config_version)
        with pytest.raises(ValueError, match="disabled"):
            hand.validate(req)
        with pytest.raises(ValueError, match="disabled"):
            arm.validate(req)
        msg.header.stamp = ros_time(1.0)
        hand.feedback("left", msg)
        assert hand.raw["left"][0].stamp == stamp
    finally:
        h.close()


@pytest.mark.parametrize("case", ["success", "no_barcode", "no_lift"])
def test_task_requires_independent_lift_barcode_and_place_evidence(tmp_path, case):
    from rclpy.callback_groups import ReentrantCallbackGroup
    from deux_task_msgs.action import RunTask
    from deux_task_runtime.common import ExclusiveAction, pause

    h = Harness(fixture(tmp_path))
    h.source.group = ReentrantCallbackGroup()
    task = h.add(TaskNode(parameter_overrides=params(config_file=str(h.config_path))))
    barcode = h.add(BarcodeBridgeNode(parameter_overrides=params(simulated=True)))
    stages = []

    async def plan(goal):
        req = goal.request
        out = PlanMotion.Result(success=True, reason="synthetic planner")
        out.plan.stage = req.stage
        out.plan.target_id = req.target_id
        out.plan.trajectory.points = [
            JointTrajectoryPoint(
                positions=[0.0] * 28, time_from_start=ros_time(0.02, Duration)
            )
        ]
        return out

    async def execute(goal):
        stage = goal.request.plan.stage
        stages.append(stage)
        if stage == "Lift" and case != "no_lift":
            h.object_z = 0.03
        if stage == "Place":
            h.object_z = 0.0
        if stage == "Retreat":
            h.separated = True
        await pause(h.source, 0.04)
        return ExecuteMotion.Result(success=True, reason="synthetic motion only")

    servers = [
        ExclusiveAction(h.source, PlanMotion, "/task/plan_motion", plan),
        ExclusiveAction(h.source, ExecuteMotion, "/task/execute_motion", execute),
    ]
    raw = h.source.create_publisher(String, "/barcode/raw", 10)
    timer = h.source.create_timer(
        0.04,
        lambda: (
            raw.publish(String(data="test-code"))
            if task.state == "WaitBarcode" and case == "success"
            else None
        ),
    )
    label_pub = h.source.create_publisher(BarcodeTarget, '/task/barcode_target', qos_profile_sensor_data)
    def label_tick():
        r = task.perception_request
        if r and r.mode == 'barcode':
            b = BarcodeTarget(request_id=r.request_id, target_id='bag', orientation_valid=True,
                              valid_fraction=1., plane_rmse_m=.001)
            b.header.stamp = h.source.get_clock().now().to_msg()
            b.header.frame_id = 'base_link'; b.pose.orientation.w = 1.
            label_pub.publish(b)
    label_timer = h.source.create_timer(.02, label_tick)
    client = ActionClient(h.source, RunTask, "/task/run")
    try:
        h.spin(
            lambda: task.observation is not None
            and task.planner.server_is_ready()
            and task.executor_client.server_is_ready()
            and client.server_is_ready()
        )
        handle = h.future(client.send_goal_async(RunTask.Goal(target_id="bag")))
        assert handle.accepted
        result = h.future(handle.get_result_async(), 15).result
        if case == "success":
            assert result.success, result.reason
            assert result.barcode == "test-code" and stages == [
                "Approach",
                "Descend",
                "Close",
                "Lift",
                "AlignBarcode",
                "Place",
                "Release",
                "Retreat",
            ]
        elif case == "no_barcode":
            assert (
                not result.success
                and stages.count("AlignBarcode") == 3
                and "Place" not in stages
            )
        else:
            assert (
                not result.success
                and stages.count("Lift") == 2
                and "AlignBarcode" not in stages
            )
    finally:
        h.close()


@pytest.mark.parametrize("cancel", ["complete", "cancel", "feedback_loss"])
def test_execution_real_bridges_with_synthetic_arm_hand_drivers(tmp_path, cancel):
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from control_msgs.action import FollowJointTrajectory
    from deux_hand_msgs.msg import HandCtlMessage
    from deux_task_runtime.common import pause
    from deux_task_runtime.trajectory import arrays, sample
    from deux_task_core.planning import hermite
    from deux_task_msgs.msg import MotionPlan

    c = fixture(tmp_path)
    h = Harness(c)
    h.source.group = ReentrantCallbackGroup()
    mapping = dict(calibrated=True, position_mode_confirmed=True)
    for side in ("left", "right"):
        names = [n for n in HAND_NAMES if n.startswith(side)]
        mapping[side] = dict(
            feedback_topic=f"/synthetic/{side}/feedback",
            feedback_type="sensor_msgs/msg/JointState",
            mode=7,
            action_state=0,
            action_state_save=0,
            joints={
                n: dict(
                    id=i,
                    feedback_name=str(i),
                    sign=-1,
                    offset=0.2,
                    min=-1.0,
                    max=1.0,
                    velocity=1.5,
                    goal_current=0.0,
                )
                for i, n in enumerate(names)
            },
        )
    path = tmp_path / "mapping.yaml"
    path.write_text(yaml.safe_dump(mapping))
    arm = h.add(
        ArmBridgeNode(
            parameter_overrides=params(
                config_file=str(h.config_path),
                hardware_enabled=True,
                driver_action="/synthetic/arm",
            )
        )
    )
    hand = h.add(
        HandBridgeNode(
            parameter_overrides=params(
                config_file=str(h.config_path),
                hardware_enabled=True,
                mapping_file=str(path),
            )
        )
    )
    execute = h.add(
        ExecutionNode(
            parameter_overrides=params(config_file=str(h.config_path), dry_run=False)
        )
    )
    driver_cancel = []
    hand_commands = []
    arm_starts = []

    async def arm_execute(goal):
        tr = goal.request.trajectory
        t, q = arrays(tr, ARM_NAMES)
        start = seconds(tr.header.stamp)
        arm_starts.append(start)
        while True:
            elapsed = h.source.get_clock().now().nanoseconds * 1e-9 - start
            if goal.is_cancel_requested:
                driver_cancel.append(True)
                goal.canceled()
                return FollowJointTrajectory.Result(error_code=-1)
            values = sample(t, q, max(0, elapsed))
            for name, value in zip(ARM_NAMES, values):
                h.q[JOINT_NAMES.index(name)] = value
            if elapsed >= t[-1]:
                goal.succeed()
                return FollowJointTrajectory.Result(error_code=0)
            await pause(h.source)

    server = ActionServer(
        h.source,
        FollowJointTrajectory,
        "/synthetic/arm",
        execute_callback=arm_execute,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
        callback_group=h.source.group,
    )

    def hand_command(side, msg):
        assert msg.mode == 7 and msg.side == side
        hand_commands.append(seconds(msg.header.stamp))
        for motor in msg.motors:
            name = [
                n for n, m in mapping[side]["joints"].items() if m["id"] == motor.id
            ][0]
            h.q[JOINT_NAMES.index(name)] = 0.2 - motor.rad

    subscribers = [
        h.source.create_subscription(
            HandCtlMessage, f"/hand/{s}/cmd", lambda m, s=s: hand_command(s, m), 10
        )
        for s in ("left", "right")
    ]
    arm_pub = h.source.create_publisher(
        JointState, "/arm/joint_states", qos_profile_sensor_data
    )
    hand_pubs = {
        s: h.source.create_publisher(
            JointState, f"/synthetic/{s}/feedback", qos_profile_sensor_data
        )
        for s in ("left", "right")
    }
    feedback_enabled = [True]

    def feedback():
        if not feedback_enabled[0]:
            return
        stamp = h.source.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = list(ARM_NAMES)
        msg.position = [float(h.q[JOINT_NAMES.index(n)]) for n in ARM_NAMES]
        arm_pub.publish(msg)
        for side, pub in hand_pubs.items():
            msg = JointState()
            msg.header.stamp = stamp
            msg.name = [str(i) for i in range(7)]
            msg.position = [
                float(0.2 - h.q[JOINT_NAMES.index(n)]) for n in mapping[side]["joints"]
            ]
            pub.publish(msg)

    timer = h.source.create_timer(0.02, feedback)
    client = ActionClient(h.source, ExecuteMotion, "/task/execute_motion")
    try:
        h.spin(
            lambda: arm.state is not None
            and len(hand.raw) == 2
            and execute.observation is not None
            and client.server_is_ready()
            and arm.client.server_is_ready()
        )
        t, q = hermite(
            [np.zeros(28), np.full(28, 0.12)], np.ones(28) * 0.3, np.ones(28), 0.02
        )
        # Extend duration to give cancellation/fault tests a meaningful in-flight interval.
        t = t * 2
        # Resample positions to the configured cadence.
        dense = np.arange(0, t[-1] + 1e-8, 0.02)
        q = np.array([sample(t, q, x) for x in dense])
        t = dense
        p = MotionPlan(
            plan_id="test-exec",
            scene_version=scene_version(c),
            target_id="bag",
            stage="Close",
            model_id=c["model_id"],
            synthetic=True,
        )
        p.header.stamp = h.source.get_clock().now().to_msg()
        p.header.frame_id = "base_link"
        p.valid_until = ros_time(seconds(p.header.stamp) + 2)
        p.target_pose = deepcopy(h.observation.objects.detections[0].pose.pose)
        p.trajectory.joint_names = list(JOINT_NAMES)
        p.trajectory.points = [
            JointTrajectoryPoint(
                positions=row.tolist(), time_from_start=ros_time(float(ti), Duration)
            )
            for ti, row in zip(t, q)
        ]
        handle = h.future(client.send_goal_async(ExecuteMotion.Goal(plan=p)))
        assert handle.accepted
        h.spin(lambda: bool(arm_starts))
        if cancel == "cancel":
            response = h.future(handle.cancel_goal_async())
            assert response.goals_canceling
        elif cancel == "feedback_loss":
            feedback_enabled[0] = False
            h.publish_enabled = False
        result = h.future(handle.get_result_async(), 10).result
        if cancel == "complete":
            assert result.success, result.reason
            assert hand_commands and min(hand_commands) >= arm_starts[0] - 0.01
            np.testing.assert_allclose(h.q, q[-1], atol=0.005)
        else:
            assert not result.success
            assert driver_cancel
    finally:
        h.close()


def test_legacy_adapter_refuses_yaw_only_and_delayed_measurements(tmp_path):
    from deux_task_runtime.observation_adapter import legacy_packet

    h = Harness(fixture(tmp_path))
    try:
        h.publish()
        obs = h.observation
        obs.left_ee_valid = True
        obs.base_from_left_ee = deepcopy(obs.base_from_right_ee)
        packet = legacy_packet(obs)
        assert packet["objects"][0]["camera"] == "external" and packet["cameras"] == {}
        obs.objects.detections[0].full_orientation = False
        with pytest.raises(ValueError, match="yaw-only"):
            legacy_packet(obs)
        obs.objects.detections[0].full_orientation = True
        obs.objects.header.stamp = ros_time(seconds(obs.header.stamp) - 1)
        with pytest.raises(ValueError, match="delayed"):
            legacy_packet(obs)
    finally:
        h.close()


def test_consumers_load_config_without_scene_topic_and_reject_version_mismatch(
    tmp_path,
):
    h = Harness(fixture(tmp_path))
    try:
        with pytest.raises(ValueError, match="config_file"):
            ExecutionNode()
        execution = h.add(
            ExecutionNode(parameter_overrides=params(config_file=str(h.config_path)))
        )
        arm = h.add(
            ArmBridgeNode(parameter_overrides=params(config_file=str(h.config_path)))
        )
        hand = h.add(
            HandBridgeNode(parameter_overrides=params(config_file=str(h.config_path)))
        )
        assert execution.config == arm.config == hand.config
        assert execution.config_version == arm.config_version == hand.config_version
        assert h.source.count_publishers("/task/scene") == 0
        assert h.source.count_subscribers("/task/scene") == 0
        for bridge in (arm, hand):
            with pytest.raises(ValueError, match="version mismatch"):
                bridge.validate(BridgeMotion.Goal(scene_version="different"))
        assert not execution.set_parameters(params(config_file="different.yaml"))[
            0
        ].successful
        version = execution.config_version
        edited = deepcopy(h.config)
        edited["lift_height"] = 0.12
        h.config_path.write_text(yaml.safe_dump(edited))
        assert (
            execution.config["lift_height"] == 0.03
            and execution.config_version == version
        )
        restarted = h.add(
            ExecutionNode(parameter_overrides=params(config_file=str(h.config_path)))
        )
        assert (
            restarted.config["lift_height"] == 0.12
            and restarted.config_version != version
        )
        with open(h.config["robot"]["urdf_path"], "a") as stream:
            stream.write("\n<!-- edited -->\n")
        with pytest.raises(ValueError, match="URDF changed"):
            execution.check_model_version()
    finally:
        h.close()


def test_alignment_guard_rejects_stale_moving_or_wrong_session_label(tmp_path):
    from deux_task_msgs.msg import MotionPlan, BarcodeTarget
    h=Harness(fixture(tmp_path))
    node=h.add(ExecutionNode(parameter_overrides=params(config_file=str(h.config_path))))
    try:
        b=BarcodeTarget(request_id='scan',target_id='bag',orientation_valid=True,valid_fraction=1.,plane_rmse_m=.001)
        b.header.stamp=h.source.get_clock().now().to_msg();b.header.frame_id='base_link';b.pose.orientation.w=1.
        p=MotionPlan(stage='AlignBarcode',target_id='bag',barcode_target=deepcopy(b))
        node.latest_barcode_target=deepcopy(b);node.barcode_guard(p)
        node.latest_barcode_target.request_id='old'
        with pytest.raises(ValueError):node.barcode_guard(p)
        node.latest_barcode_target=deepcopy(b);node.latest_barcode_target.pose.position.x=.5
        with pytest.raises(ValueError,match='moved'):node.barcode_guard(p)
        node.latest_barcode_target=deepcopy(b);node.latest_barcode_target.header.stamp=ros_time(1.)
        with pytest.raises(ValueError):node.barcode_guard(p)
    finally:h.close()


def test_simulation_wall_watchdog_requires_sim_clock(tmp_path):
    c=fixture(tmp_path);c['simulation_wall_timeout_scale']=4.
    h=Harness(c)
    try:
        from deux_task_runtime.common import RuntimeNode
        with pytest.raises(ValueError,match='use_sim_time'):
            RuntimeNode('bad_clock',parameter_overrides=params(config_file=str(h.config_path)))
        node=h.add(RuntimeNode('sim_clock',parameter_overrides=params(config_file=str(h.config_path),use_sim_time=True)))
        assert node.wall_timeout(2.) == 8.
    finally:h.close()
