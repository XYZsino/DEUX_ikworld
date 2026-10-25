"""Mocked ROS driver; run only on isolated ROS_DOMAIN_ID=198."""

import json, multiprocessing, os, signal, subprocess, sys, time
from pathlib import Path


def mock_driver(ready, eventfile):
    import numpy as np, rclpy
    from rclpy.node import Node
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.action import ActionServer, CancelResponse
    from sensor_msgs.msg import JointState
    from control_msgs.msg import JointTrajectoryControllerState
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers
    from controller_manager_msgs.msg import ControllerState
    from rcl_interfaces.srv import GetParameters
    from rcl_interfaces.msg import ParameterValue
    from std_msgs.msg import String
    from arm_step_core import ARM_NAMES

    rclpy.init()
    n = Node("mock_arm_driver")
    group = ReentrantCallbackGroup()
    q = np.zeros(14)
    events = []
    jp = n.create_publisher(JointState, "/arm/joint_states", 10)
    cp = n.create_publisher(
        JointTrajectoryControllerState, "/arm/deux_arm_controller/controller_state", 10
    )
    sp = n.create_publisher(String, "/deux/stiffness/state", 10)

    def tick():
        stamp = n.get_clock().now().to_msg()
        j = JointState()
        j.header.stamp = stamp
        j.name = list(ARM_NAMES)
        j.position = q.tolist()
        jp.publish(j)
        c = JointTrajectoryControllerState()
        c.header.stamp = stamp
        c.joint_names = list(ARM_NAMES)
        c.reference.positions = q.tolist()
        c.feedback.positions = q.tolist()
        cp.publish(c)
        sp.publish(
            String(
                data=json.dumps(
                    {
                        "t": stamp.sec + stamp.nanosec * 1e-9,
                        "estop": False,
                        "applied": "setpoint",
                        "mode": "mock_only",
                    }
                )
            )
        )

    n.create_timer(0.02, tick)
    urdf = (
        "<robot>"
        + "".join(
            f'<joint name="{a}" type="revolute"><limit lower="-1" upper="1"/></joint>'
            for a in ARM_NAMES
        )
        + "</robot>"
    )

    def description(req, res):
        res.values = [ParameterValue(type=4, string_value=urdf)]
        return res

    def controllers(req, res):
        res.controller = [
            ControllerState(
                name="deux_arm_controller",
                type="joint_trajectory_controller/JointTrajectoryController",
                state="active",
            )
        ]
        return res

    n.create_service(
        GetParameters, "/arm/robot_state_publisher/get_parameters", description
    )
    n.create_service(
        ListControllers, "/arm/controller_manager/list_controllers", controllers
    )

    def record(value):
        events.append(value)
        Path(eventfile).write_text(json.dumps(events))

    def execute(goal):
        record("accepted")
        tr = goal.request.trajectory
        start = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
        ts = [
            p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in tr.points
        ]
        poses = np.array([p.positions for p in tr.points])
        abort_begin = time.time()
        abort_test = events.count("accepted") == 3
        while True:
            if abort_test and time.time() - abort_begin > 0.3:
                goal.abort()
                record("aborted_terminal")
                return FollowJointTrajectory.Result(
                    error_code=-5, error_string="mock final goal miss"
                )
            if goal.is_cancel_requested:
                record("cancel_requested")
                time.sleep(0.25)
                goal.canceled()
                record("canceled_terminal")
                return FollowJointTrajectory.Result(error_code=0)
            elapsed = n.get_clock().now().nanoseconds * 1e-9 - start
            q[:] = [np.interp(max(0, elapsed), ts, poses[:, i]) for i in range(14)]
            if elapsed >= ts[-1]:
                goal.succeed()
                record("succeeded")
                return FollowJointTrajectory.Result(error_code=0)
            time.sleep(0.01)

    server = ActionServer(
        n,
        FollowJointTrajectory,
        "/arm/deux_arm_controller/follow_joint_trajectory",
        execute_callback=execute,
        cancel_callback=lambda _: CancelResponse.ACCEPT,
        callback_group=group,
    )
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    ready.set()
    try:
        ex.spin()
    finally:
        server.destroy()
        ex.shutdown()
        n.destroy_node()
        rclpy.try_shutdown()


def test_preview_execute_and_cancel(tmp_path):
    assert (
        os.environ.get("ROS_DOMAIN_ID") == "198"
    ), "Mock test requires isolated domain 198"
    assert (
        os.environ.get("ROS_LOCALHOST_ONLY") == "1"
        or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST"
    ), "Mock requires localhost-only discovery"
    ctx = multiprocessing.get_context("spawn")
    ready = ctx.Event()
    events = tmp_path / "events.json"
    driver = ctx.Process(target=mock_driver, args=(ready, str(events)))
    driver.start()
    base = [
        sys.executable,
        str(Path(__file__).resolve().parents[1] / "arm_step_check.py"),
    ]
    try:
        assert ready.wait(10)
        for label, cancel in [
            ("complete", False),
            ("cancel", True),
            ("aborted", False),
        ]:
            preview = tmp_path / (label + "_preview.json")
            output = tmp_path / (label + "_result.json")
            run = subprocess.run(
                base + ["--output", str(preview)],
                timeout=15,
                capture_output=True,
                text=True,
            )
            assert run.returncode == 0, run.stdout + run.stderr
            info = json.loads(preview.read_text())
            assert not info["goal_sent"] and not info["blockers"], info
            proc = subprocess.Popen(
                base
                + ["--output", str(output), "--execute-reviewed-plan", str(preview)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            if cancel:
                end = time.monotonic() + 10
                while time.monotonic() < end:
                    if (
                        events.exists()
                        and json.loads(events.read_text()).count("accepted") == 2
                    ):
                        break
                    time.sleep(0.05)
                else:
                    raise AssertionError("no second goal accepted")
                time.sleep(0.15)
                proc.send_signal(signal.SIGINT)
            log, _ = proc.communicate(timeout=15)
            assert output.exists(), log
            result = json.loads(output.read_text())
            assert result["goal_sent"] and result.get("stop_confirmed"), log
            if cancel:
                assert (
                    proc.returncode == 1
                    and result["canceled_after_failure"]
                    and "canceled_terminal" in json.loads(events.read_text())
                ), log
            elif label == "aborted":
                assert (
                    proc.returncode == 1
                    and result["terminal_status"] == 6
                    and result["canceled_after_failure"] is False
                ), log
                assert (
                    result["cleanup"]["cancel_requested"] is False
                    and "mock final goal miss" in result["primary_error"]
                ), log
            else:
                assert proc.returncode == 0 and result["driver_status"] == 4, log
    finally:
        driver.terminate()
        driver.join(timeout=5)
        if driver.is_alive():
            driver.kill()
            driver.join()
