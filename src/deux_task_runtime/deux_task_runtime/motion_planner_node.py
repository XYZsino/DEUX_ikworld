from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from copy import deepcopy
from uuid import uuid4
import time
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.action import PlanMotion
from deux_task_core.model import JOINT_NAMES
from deux_task_core.planning import Planner, STAGES, scene_version
from .common import RuntimeNode, ExclusiveAction, pose_matrix, ros_time, pause, run, current


def compute(config, version, stage, start, target, barcode=None):
    if scene_version(config) != version:
        raise ValueError("URDF changed before planning")
    planner = Planner(config)
    if scene_version(config) != version:
        raise ValueError("URDF changed while loading planner")
    return planner.plan(stage, start, target, barcode_pose=barcode)


class MotionPlannerNode(RuntimeNode):
    def __init__(self, **kwargs):
        super().__init__("motion_planner_node", **kwargs)
        self.pool = ProcessPoolExecutor(
            max_workers=1, mp_context=multiprocessing.get_context("spawn")
        )
        self.work = None
        self.action = ExclusiveAction(
            self, PlanMotion, "/task/plan_motion", self.execute, self.validate
        )

    def validate_label(self, req):
        b = req.barcode_target
        if (not b.request_id or b.target_id != req.target_id or not b.orientation_valid
                or b.header.frame_id != self.config['base_frame']
                or not current(b.header.stamp, self.now(), self.config['max_object_age'])
                or not np.isfinite([b.plane_rmse_m,b.valid_fraction]).all()
                or not 0 <= b.plane_rmse_m <= .003 or not .7 <= b.valid_fraction <= 1.):
            raise ValueError('fresh associated barcode surface required')
        return pose_matrix(b.pose)

    def validate(self, request):
        self.check_model_version()
        if request.stage == "AlignBarcode": self.validate_label(request)
        if self.work is not None and not self.work.done():
            raise ValueError("previous planner still computing")
        if request.stage not in STAGES or self.config is None:
            raise ValueError("scene/stage unavailable")
        if request.scene_version != self.config_version:
            raise ValueError("scene version changed")
        self.measured(request.observation)
        self.target(
            request.target_id,
            request.observation,
            full=request.stage in ("Scan", "Place"),
        )

    async def execute(self, goal):
        req = goal.request
        c = deepcopy(self.config)
        version = self.config_version
        start = self.measured(req.observation)
        obj = self.target(
            req.target_id, req.observation, full=req.stage in ("Scan", "Place")
        )
        self.work = self.pool.submit(
            compute, c, version, req.stage, start, pose_matrix(obj.pose.pose),
            self.validate_label(req) if req.stage == "AlignBarcode" else None
        )
        wall = time.monotonic()
        clock = self.now()
        while not self.work.done():
            if goal.is_cancel_requested:
                raise InterruptedError("planning canceled")
            if self.config_version != version or self.now() < clock:
                raise ValueError("scene/clock changed")
            if time.monotonic() - wall > 30:
                raise TimeoutError("planning deadline")
            await pause(self)
        data = self.work.result()
        if req.stage == "AlignBarcode": self.validate_label(req)
        latest_target = self.target(req.target_id, full=req.stage in ("Scan", "Place"))
        self.check_target_rotation(latest_target.pose.pose, obj.pose.pose)
        if (
            np.linalg.norm(
                pose_matrix(latest_target.pose.pose)[:3, 3]
                - pose_matrix(obj.pose.pose)[:3, 3]
            )
            > c["max_target_drift"]
        ):
            raise ValueError("target moved during planning")
        # Revalidate snapshot after the expensive computation.
        latest = self.measured()
        if max(abs(latest - start)) > c["start_tolerance"]:
            raise ValueError("robot moved during planning")
        if self.config_version != version:
            raise ValueError("scene version changed")
        result = PlanMotion.Result(success=True, reason="planned")
        plan = result.plan
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.header.frame_id = c["base_frame"]
        plan.plan_id = uuid4().hex
        plan.scene_version = version
        plan.target_id = req.target_id
        plan.stage = req.stage
        plan.model_id = c["model_id"]
        plan.synthetic = bool(c.get("synthetic", False))
        plan.valid_until = ros_time(self.now() + c["max_plan_age"])
        plan.target_pose = obj.pose.pose
        plan.barcode_target = req.barcode_target
        plan.trajectory.joint_names = list(JOINT_NAMES)
        plan.ik_calls = data["ik_calls"]
        plan.minimum_clearance = data["minimum_clearance"]
        for t, q in zip(data["times"], data["path"]):
            plan.trajectory.points.append(
                JointTrajectoryPoint(
                    positions=q.tolist(), time_from_start=ros_time(t, Duration)
                )
            )
        return result

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    run(MotionPlannerNode, args)
