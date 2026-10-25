from copy import deepcopy
import time
import numpy as np
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from std_msgs.msg import String
from deux_task_msgs.action import ExecuteMotion, BridgeMotion
from deux_task_core.model import JOINT_NAMES, ARM_NAMES, HAND_NAMES, RobotModel
from .common import (
    RuntimeNode,
    ExclusiveAction,
    seconds,
    ros_time,
    pose_matrix,
    pause,
    wait,
    cancel_children,
    run,
    current,
)
from .trajectory import arrays, sample


class ExecutionNode(RuntimeNode):
    def __init__(self, **kwargs):
        super().__init__("execution_node", observe_state=True, observe_barcode=True, observe_status=True, **kwargs)
        self.declare_parameter("dry_run", True)
        self.bridge_clients = [
            ActionClient(self, BridgeMotion, name, callback_group=self.group)
            for name in ("/task/arm_motion", "/task/hand_motion")
        ]
        self.record = self.create_publisher(String, "/task/execution_record", 10)
        self.action = ExclusiveAction(
            self, ExecuteMotion, "/task/execute_motion", self.execute, self.validate
        )
        self.completed = set()

    def barcode_guard(self, plan):
        observed, expected = self.latest_barcode_target, plan.barcode_target
        if (observed is None or not expected.request_id
                or observed.request_id != expected.request_id
                or observed.target_id != plan.target_id or expected.target_id != plan.target_id
                or observed.header.frame_id != self.config['base_frame']
                or expected.header.frame_id != self.config['base_frame']
                or not expected.orientation_valid or not observed.orientation_valid
                or not current(observed.header.stamp, self.now(), self.config['max_object_age'])
                or seconds(observed.header.stamp) < seconds(expected.header.stamp)
                or not np.isfinite([observed.plane_rmse_m, observed.valid_fraction]).all()
                or not 0 <= observed.plane_rmse_m <= .003 or not .7 <= observed.valid_fraction <= 1.):
            raise ValueError('barcode alignment observation missing/stale/superseded')
        self.check_target_rotation(observed.pose, expected.pose)
        if np.linalg.norm(pose_matrix(observed.pose)[:3,3]-pose_matrix(expected.pose)[:3,3]) > self.config['max_target_drift']:
            raise ValueError('barcode moved; replan')

    def validate(self, request):
        self.check_model_version()
        p = request.plan
        if p.stage == "AlignBarcode": self.barcode_guard(p)
        q = self.measured()
        if (
            self.config_version != p.scene_version
            or self.config["model_id"] != p.model_id
        ):
            raise ValueError("scene/model changed")
        if not p.plan_id or p.plan_id in self.completed:
            raise ValueError("duplicate/empty plan")
        if not seconds(p.header.stamp) <= self.now() < seconds(p.valid_until):
            raise ValueError("plan expired/future")
        t, path = arrays(p.trajectory, JOINT_NAMES)
        if np.max(abs(path[0] - q)) > self.config["start_tolerance"]:
            raise ValueError("start mismatch")
        obj = self.target(p.target_id, full=p.stage in ("Scan", "Place"))
        self.check_target_rotation(obj.pose.pose, p.target_pose)
        if (
            np.linalg.norm(
                pose_matrix(obj.pose.pose)[:3, 3] - pose_matrix(p.target_pose)[:3, 3]
            )
            > self.config["max_target_drift"]
        ):
            raise ValueError("target moved; replan")
        if (
            p.synthetic
            and not self.get_parameter("dry_run").value
            and not self.config.get("synthetic", False)
        ):
            raise ValueError("synthetic plan in hardware scene")

    async def execute(self, goal):
        self.validate(goal.request)
        plan = goal.request.plan
        c = deepcopy(self.config)
        t, q = arrays(plan.trajectory, JOINT_NAMES)
        if not self.get_parameter("dry_run").value and not (
            c.get("synthetic", False)
            or (
                c.get("calibration_confirmed") is True
                and c.get("command_ownership_confirmed") is True
            )
        ):
            raise ValueError("installation calibration/command ownership unconfirmed")
        robot = RobotModel(c["robot"])
        if np.any(
            abs(np.diff(q, axis=0) / np.diff(t)[:, None]) > robot.velocity * 1.001
        ):
            raise ValueError("velocity limit")
        velocity = np.diff(q, axis=0) / np.diff(t)[:, None]
        accel = (
            np.diff(np.vstack([np.zeros(28), velocity, np.zeros(28)]), axis=0)
            / np.r_[np.diff(t), np.diff(t)[-1]][:, None]
        )
        if np.any(abs(accel) > robot.acceleration * 1.01):
            raise ValueError("acceleration limit")
        if np.any(np.diff(t) > c["sample_period"] * 1.01):
            raise ValueError("trajectory sampling too sparse")
        attached = (
            c["attached_radius"]
            if plan.stage in ("Lift", "Scan", "AlignBarcode", "Place", "Recover")
            else 0.0
        )
        for row in q:
            robot.clearance(row, c["obstacles"], c["collision_margin"], attached)
        self.validate(goal.request)
        self.completed.add(plan.plan_id)
        if self.get_parameter("dry_run").value:
            self.record.publish(
                String(
                    data=f"{plan.plan_id}: {plan.stage}: checked {len(q)} samples; no dispatch"
                )
            )
            return ExecuteMotion.Result(success=True, reason="dry_run_only")
        self.hardware_gate()
        if not all(client.server_is_ready() for client in self.bridge_clients):
            raise ValueError("bridges unavailable")
        handles = []
        pending = []
        start = self.now() + 0.5
        deadline = start + t[-1] + c["settle_timeout"]
        try:
            for client, names in zip(self.bridge_clients, (ARM_NAMES, HAND_NAMES)):
                tr = deepcopy(plan.trajectory)
                indices = [JOINT_NAMES.index(n) for n in names]
                tr.joint_names = list(names)
                tr.header.stamp = ros_time(start)
                for p in tr.points:
                    p.positions = [p.positions[i] for i in indices]
                pending.append(
                    client.send_goal_async(
                        BridgeMotion.Goal(
                            plan_id=plan.plan_id,
                            scene_version=plan.scene_version,
                            trajectory=tr,
                            deadline=ros_time(deadline),
                        )
                    )
                )
            for future in pending:
                handle = await wait(
                    self, future, max(0.01, start - self.now() - 0.1), goal
                )
                handles.append(handle)
                if not handle.accepted:
                    raise ValueError("bridge rejected")
            if self.now() >= start:
                raise TimeoutError("bridge acceptance missed synchronized start")
            results = [h.get_result_async() for h in handles]
            wall = time.monotonic()
            previous = self.now()
            while not all(r.done() for r in results):
                now = self.now()
                if goal.is_cancel_requested:
                    raise InterruptedError("canceled")
                if (
                    now < previous
                    or now > deadline
                    or time.monotonic() - wall > self.wall_timeout(t[-1] + c["settle_timeout"] + 3)
                ):
                    raise TimeoutError("execution clock/deadline")
                previous = now
                self.hardware_gate()
                if self.config_version != plan.scene_version:
                    raise ValueError("scene changed")
                measured = self.measured()
                if plan.stage == "AlignBarcode": self.barcode_guard(plan)
                if (
                    np.max(abs(measured - sample(t, q, max(0, now - start))))
                    > c["tracking_tolerance"]
                ):
                    raise ValueError("tracking error")
                if plan.stage in ("Approach", "Descend"):
                    obj = self.target(plan.target_id)
                    self.check_target_rotation(obj.pose.pose, plan.target_pose)
                    if (
                        np.linalg.norm(
                            pose_matrix(obj.pose.pose)[:3, 3]
                            - pose_matrix(plan.target_pose)[:3, 3]
                        )
                        > c["max_target_drift"]
                    ):
                        raise ValueError("target moved during approach")
                for result in results:
                    if result.done() and (
                        result.result().status != GoalStatus.STATUS_SUCCEEDED
                        or not result.result().result.success
                    ):
                        raise ValueError(
                            "bridge failed: " + result.result().result.reason
                        )
                goal.publish_feedback(
                    ExecuteMotion.Feedback(
                        progress=float(np.clip((now - start) / t[-1], 0, 1)),
                        status=plan.stage,
                    )
                )
                await pause(self)
            if any(
                r.result().status != GoalStatus.STATUS_SUCCEEDED
                or not r.result().result.success
                for r in results
            ):
                raise ValueError("bridge result failure")
            if np.max(abs(self.measured() - q[-1])) > c["start_tolerance"]:
                raise ValueError("final measured position mismatch")
            return ExecuteMotion.Result(
                success=True,
                reason="measured motion complete; task outcome requires sensor evidence",
            )
        finally:
            # Late acceptances are also canceled if another bridge rejects/times out.
            for future in pending:
                if future.done():
                    h = future.result()
                    if h not in handles:
                        handles.append(h)
                else:
                    future.add_done_callback(
                        lambda f: (
                            f.result().cancel_goal_async()
                            if f.result().accepted
                            else None
                        )
                    )
            await cancel_children(self, handles)


def main(args=None):
    run(ExecutionNode, args)
