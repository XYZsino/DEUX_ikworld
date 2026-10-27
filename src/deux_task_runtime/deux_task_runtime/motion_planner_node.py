from deux_task_core.planning import requires_object_pose
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from copy import deepcopy
from uuid import uuid4
import time
import json
import numpy as np
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.action import PlanMotion
from deux_task_core.model import JOINT_NAMES
from deux_task_core.planning import Planner, STAGES, scene_version
from deux_task_core.ik_backends import backend_name, warmup
from .common import RuntimeNode, ExclusiveAction, pose_matrix, ros_time, pause, run, current


_planner_cache = None


def cached_planner(config, version):
    global _planner_cache
    if scene_version(config) != version:
        raise ValueError("URDF changed before planning")
    if _planner_cache is None or _planner_cache[0] != version:
        _planner_cache = (version, Planner(config))
    planner = _planner_cache[1]
    if scene_version(config) != version:
        raise ValueError("URDF changed while loading planner")
    planner.robot.ik_calls = 0
    return planner


def prepare_backend(config, version):
    planner=cached_planner(config,version)
    warmup(planner)
    return backend_name(config)


def compute(config, version, stage, start, target, barcode=None, motion=None, candidate_id=''):
    data=compute_plan(config,version,stage,start,target,barcode,motion,candidate_id)
    data['ik_backend']=backend_name(config)
    if 'candidate_report' in data:data['candidate_report']['ik_backend']=data['ik_backend']
    return data


def compute_plan(config, version, stage, start, target, barcode=None, motion=None, candidate_id=''):
    planner=cached_planner(config,version)
    if motion is not None:
        from deux_task_core.grasp_candidates import settings, select_candidate, revalidate_grasp
        if settings(config).get('enabled',False):
            if stage=='Approach':return select_candidate(planner,start,target,motion[0],motion[1],candidate_id)
            if stage=='TrackGrasp':return revalidate_grasp(planner,candidate_id,start,target,motion[0],motion[1])
        from deux_task_core.conveyor import plan_conveyor,plan_track_grasp
        return plan_track_grasp(planner,start,target,motion[0],motion[1]) if stage=='TrackGrasp' else plan_conveyor(planner,stage,start,target,motion[0],motion[1])
    return planner.plan(stage, start, target, barcode_pose=barcode)


class MotionPlannerNode(RuntimeNode):
    def __init__(self, **kwargs):
        super().__init__("motion_planner_node", observe_state=True, **kwargs)
        self.subscribe_joints()
        self.candidate_pub=self.create_publisher(String,'/task/grasp_candidates',10)
        self.pool = ProcessPoolExecutor(
            max_workers=1, mp_context=multiprocessing.get_context("spawn")
        )
        self.work = None
        if self.config and backend_name(self.config)!='scipy':
            self.get_logger().info(f'Warming {backend_name(self.config)} IK before accepting tasks')
            self.pool.submit(prepare_backend,self.config,self.config_version).result(timeout=180.)
        if self.config:self.get_logger().info(f'IK backend ready: {backend_name(self.config)}; common trajectory/collision validation')
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

    def grasp_pose_age(self, stage):
        if self.config.get('moving_source',False) and stage in ('Approach','TrackGrasp') and self.config['conveyor_tracking'].get('occluded_grasp_enabled',False):
            cfg=self.config['conveyor_tracking']
            return cfg.get('approach_prediction_max_age',self.config['max_object_age']) if stage=='Approach' else cfg['occlusion_prediction_max_age']
        if not self.config.get('moving_source',False) and self.config.get('perception_policy')=='head_rgbd' and stage in ('Approach','Descend','Close'):
            return self.config['grasp_pose_max_age']
        return None

    def validate(self, request):
        self.check_model_version()
        if request.stage == "AlignBarcode": self.validate_label(request)
        if self.work is not None and not self.work.done():
            raise ValueError("previous planner still computing")
        if request.stage not in STAGES or self.config is None:
            raise ValueError("scene/stage unavailable")
        if request.scene_version != self.config_version:
            raise ValueError("scene version changed")
        from deux_task_core.grasp_candidates import settings, candidate_config
        if settings(self.config).get('enabled',False) and request.stage=='TrackGrasp':
            candidate_config(self.config,request.grasp_candidate_id)
        self.measured()
        if requires_object_pose(self.config, request.stage):
            self.target(
                request.target_id,
                request.observation,
                full=request.stage in ("Scan", "Place"), max_age=self.grasp_pose_age(request.stage),
            )

    async def execute(self, goal):
        req = goal.request
        c = deepcopy(self.config)
        version = self.config_version
        start = self.measured()
        obj = None if not requires_object_pose(c, req.stage) else self.target(
            req.target_id, req.observation, full=req.stage in ("Scan", "Place"), max_age=self.grasp_pose_age(req.stage)
        )
        target_pose = np.eye(4) if obj is None else pose_matrix(obj.pose.pose)
        moving=c.get('moving_source',False) and req.stage in ('Approach','Descend','Close','TrackGrasp')
        motion=None
        if moving:
            if not obj.pose_filtered or not obj.velocity_valid:raise ValueError('stable filtered conveyor velocity required')
            v=obj.velocity;velocity=[v.linear.x,v.linear.y,v.linear.z,v.angular.z]
            from deux_task_core.conveyor import conveyor_velocity
            velocity=conveyor_velocity(velocity,c).tolist()
            from deux_task_core.grasp_candidates import settings
            search=settings(c)
            launch_lead=(search['selection_start_lead_s'] if req.stage=='Approach' and search.get('enabled',False)
                         else c['conveyor_tracking']['start_lead'])
            scheduled=self.now()+launch_lead
            from .common import seconds
            motion=(velocity,scheduled-seconds(req.observation.objects.header.stamp))
        self.work = self.pool.submit(
            compute, c, version, req.stage, start, target_pose,
            self.validate_label(req) if req.stage == "AlignBarcode" else None,motion,req.grasp_candidate_id
        )
        wall = time.monotonic()
        clock = self.now()
        refresh_candidate = moving and req.stage=='Approach' and search.get('enabled',False)
        acquired_at=deepcopy(req.observation.objects.header.stamp)
        while True:
            while not self.work.done():
                if goal.is_cancel_requested:
                    raise InterruptedError("planning canceled")
                if self.config_version != version or self.now() < clock:
                    raise ValueError("scene/clock changed")
                if time.monotonic() - wall > 30:
                    raise TimeoutError("planning deadline")
                await pause(self)
            if goal.is_cancel_requested:raise InterruptedError('planning canceled')
            if self.config_version != version or self.now()<clock:raise ValueError('scene/clock changed')
            data = self.work.result()
            if 'candidate_report' in data:
                report=dict(data['candidate_report'],target_id=req.target_id,stage=req.stage,
                            scene_version=version,scheduled_start_s=scheduled,computed_at_s=self.now(),
                            acquired_at_s=seconds(acquired_at),comparison_only=refresh_candidate)
                self.candidate_pub.publish(String(data=json.dumps(report,allow_nan=False)))
                self.get_logger().info(f"grasp candidates: {report.get('selected_candidate_id') or 'none'}; {report['scope']}")
            if data.get('planning_error'):raise ValueError(data['planning_error'])
            if not refresh_candidate:break
            # Candidate comparison is not an executable deadline. Reacquire a
            # fresh snapshot and revalidate the chosen full chain at a NEW start.
            latest_obs=deepcopy(self.observation)
            obj=self.target(req.target_id,latest_obs)
            if not obj.pose_filtered or not obj.velocity_valid:raise ValueError('fresh filtered conveyor velocity required after search')
            latest_start=self.measured()
            if max(abs(latest_start-start))>c['start_tolerance']:raise ValueError('robot moved during candidate search')
            start=latest_start;target_pose=pose_matrix(obj.pose.pose)
            acquired_at=deepcopy(latest_obs.objects.header.stamp)
            v=obj.velocity
            velocity=conveyor_velocity([v.linear.x,v.linear.y,v.linear.z,v.angular.z],c).tolist()
            scheduled=self.now()+c['conveyor_tracking']['start_lead']
            motion=(velocity,scheduled-seconds(acquired_at))
            self.work=self.pool.submit(compute,c,version,req.stage,start,target_pose,None,motion,data['candidate_id'])
            refresh_candidate=False
        if req.stage == "AlignBarcode": self.validate_label(req)
        if moving and self.now()>scheduled-.1:raise ValueError('moving plan missed scheduled start; replan')
        if requires_object_pose(c, req.stage) and self.grasp_pose_age(req.stage) is None and not moving:
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
        result = PlanMotion.Result(success=True, reason=f"planned (IK: {data['ik_backend']})")
        plan = result.plan
        plan.header.stamp = self.get_clock().now().to_msg()
        plan.header.frame_id = c["base_frame"]
        plan.plan_id = uuid4().hex
        plan.scene_version = version
        plan.target_id = req.target_id
        plan.stage = req.stage
        plan.ik_backend = data['ik_backend']
        plan.grasp_candidate_id = data.get('candidate_id','')
        plan.model_id = c["model_id"]
        plan.synthetic = bool(c.get("synthetic", False))
        plan.valid_until = ros_time(self.now() + c["max_plan_age"])
        if obj is not None:
            plan.target_pose = obj.pose.pose
            plan.target_acquired_at = acquired_at
        else:
            plan.target_pose.orientation.w = 1.0
        plan.barcode_target = req.barcode_target
        plan.trajectory.joint_names = list(JOINT_NAMES)
        if moving:
            plan.follows_conveyor=True;plan.target_velocity=deepcopy(obj.velocity)
            plan.target_velocity.linear.x,plan.target_velocity.linear.y,plan.target_velocity.linear.z=map(float,velocity[:3])
            plan.target_velocity.angular.z=float(velocity[3])
            plan.phase_end_times=data.get('phase_end_times',[]).tolist() if 'phase_end_times' in data else []
            plan.trajectory.header.stamp=ros_time(scheduled)
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
