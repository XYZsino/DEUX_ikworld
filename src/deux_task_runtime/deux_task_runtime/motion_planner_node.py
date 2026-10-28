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
from .common import RuntimeNode, ExclusiveAction, pose_matrix, ros_time, pause, run, current, seconds


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


def compute(config, version, stage, start, target, barcode=None, motion=None, candidate_id='', timing=None, covariance=None):
    data=compute_plan(config,version,stage,start,target,barcode,motion,candidate_id,timing,covariance)
    data['ik_backend']=backend_name(config)
    if 'candidate_report' in data:
        data['candidate_report']['ik_backend']=data['ik_backend']
        if timing is not None:data['candidate_report']['time_search']=True
    return data


def compute_plan(config, version, stage, start, target, barcode=None, motion=None, candidate_id='', timing=None, covariance=None):
    planner=cached_planner(config,version)
    if motion is not None:
        from deux_task_core.grasp_candidates import settings, select_candidate, revalidate_grasp
        if settings(config).get('enabled',False):
            from deux_task_core.time_candidates import settings as time_settings,search,revalidate
            if stage=='Approach' and time_settings(config).get('enabled',False):
                return search(planner,start,target,motion[0],motion[1],covariance) if timing is None else revalidate(planner,start,target,motion[0],motion[1],candidate_id,timing,covariance)
            if stage=='Approach':return select_candidate(planner,start,target,motion[0],motion[1],candidate_id)
            if stage=='TrackGrasp':return revalidate_grasp(planner,candidate_id,start,target,motion[0],motion[1],timing)
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
        from deux_task_core.planning_latency import PlanningLatency
        self.planning_latency = PlanningLatency(self.config or {}, self.get_parameter('use_sim_time').value)
        self.latency_scene_version = self.config_version
        self.latency_timer = self.create_timer(.1, self.sample_planning_clock)
        self.action = ExclusiveAction(
            self, PlanMotion, "/task/plan_motion", self.execute, self.validate
        )
        from .grasp_prefetch import GraspPrefetch
        self.grasp_prefetch=GraspPrefetch(self)
        self.prefetch_timer=self.create_timer(.05,self.grasp_prefetch.poll)

    def sample_planning_clock(self):
        self.planning_latency.sample(self.now(), time.monotonic())

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
        if request.stage!='TrackGrasp':self.grasp_prefetch.clear()
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
        motion=None;timing=None;covariance=None;timed_approach=False
        if moving:
            if not obj.pose_filtered or not obj.velocity_valid:raise ValueError('stable filtered conveyor velocity required')
            v=obj.velocity;velocity=[v.linear.x,v.linear.y,v.linear.z,v.angular.z]
            from deux_task_core.conveyor import conveyor_velocity
            velocity=conveyor_velocity(velocity,c).tolist()
            from deux_task_core.grasp_candidates import settings
            search=settings(c)
            launch_lead=(search['selection_start_lead_s'] if req.stage=='Approach' and search.get('enabled',False)
                         else c['conveyor_tracking']['start_lead'])
            from deux_task_core.time_candidates import settings as time_settings
            timed=time_settings(c).get('enabled',False)
            timed_approach=timed and req.stage=='Approach' and search.get('enabled',False)
            if self.latency_scene_version != version:
                from deux_task_core.planning_latency import PlanningLatency
                self.planning_latency=PlanningLatency(c,self.get_parameter('use_sim_time').value)
                self.latency_scene_version=version
            self.sample_planning_clock()
            if obj.motion_covariance_valid:covariance=list(obj.motion_covariance)
            scheduled=self.now()+launch_lead
            if timed_approach:
                reservation=self.planning_latency.diagnostics()
                scheduled=self.planning_latency.reserve_start(self.now())
            if timed and req.stage=='TrackGrasp':
                scheduled=seconds(req.grasp_start_at)
                if scheduled<self.now()+.1:raise ValueError('selected grasp start deadline elapsed')
                timing=dict(phase_durations=list(req.grasp_phase_durations))
            motion=(velocity,scheduled-seconds(req.observation.objects.header.stamp))
        reuse_path=timed_approach and time_settings(c).get('reuse_validated_path',False)
        prefetched=None
        if moving and req.stage=='TrackGrasp' and time_settings(c).get('prefetch_grasp',False):
            prefetched=self.grasp_prefetch.take(req,start)
            self.work=prefetched['future']
            obj=prefetched['obj'];target_pose=prefetched['target'];velocity=prefetched['velocity']
            motion=(velocity,scheduled-seconds(prefetched['acquired']))
        else:
            self.work = self.pool.submit(
                compute, c, version, req.stage, start, target_pose,
                self.validate_label(req) if req.stage == "AlignBarcode" else None,motion,req.grasp_candidate_id,timing,covariance
            )
        wall = time.monotonic()
        clock = self.now()
        work_wall=wall;work_clock=clock;research_attempt=0
        refresh_candidate = moving and req.stage=='Approach' and search.get('enabled',False)
        acquired_at=deepcopy(prefetched['acquired'] if prefetched else req.observation.objects.header.stamp)

        def refresh_inputs():
            nonlocal start,target_pose,acquired_at,obj,velocity,covariance,motion
            latest_obs=deepcopy(self.observation)
            obj=self.target(req.target_id,latest_obs)
            if not obj.pose_filtered or not obj.velocity_valid:
                raise ValueError('fresh filtered conveyor velocity required after search')
            latest_start=self.measured()
            if max(abs(latest_start-start))>c['start_tolerance']:
                raise ValueError('robot moved during candidate search')
            start=latest_start;target_pose=pose_matrix(obj.pose.pose)
            acquired_at=deepcopy(latest_obs.objects.header.stamp)
            v=obj.velocity
            velocity=conveyor_velocity([v.linear.x,v.linear.y,v.linear.z,v.angular.z],c).tolist()
            covariance=list(obj.motion_covariance) if obj.motion_covariance_valid else None
            motion=(velocity,scheduled-seconds(acquired_at))

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
            if time.monotonic()-wall>30:raise TimeoutError('planning deadline')
            data = self.work.result()
            if prefetched:
                self.grasp_prefetch.guard(prefetched,req,self.measured())
                if 'candidate_report' in data:
                    data['candidate_report']['prefetch']=dict(
                        submitted_at_s=prefetched['submitted_at'],
                        compute_wall_s=prefetched.get('completed_wall',time.monotonic())-prefetched['submitted_wall'],
                        request_wait_wall_s=time.monotonic()-wall,remaining_s=scheduled-self.now())
                self.get_logger().info(f'Grasp prefetch reused: request_wait_wall_s={time.monotonic()-wall:.3f}, remaining_s={scheduled-self.now():.3f}')
            if timed_approach:
                self.sample_planning_clock()
                self.planning_latency.observe('search' if refresh_candidate else 'revalidation',time.monotonic()-work_wall)
                in_time=(self.planning_latency.can_validate(scheduled,self.now()) if refresh_candidate and not reuse_path
                         else self.planning_latency.can_dispatch(scheduled,self.now()))
                if 'candidate_report' in data:
                    data['candidate_report']['planning_budget']=dict(self.planning_latency.diagnostics(),
                        measured_wall_s=time.monotonic()-work_wall,measured_clock_s=self.now()-work_clock,
                        remaining_before_start_s=scheduled-self.now(),research_attempt=research_attempt,
                        schedule_state='reserved' if in_time else 'expired_research_required',reservation=reservation)
            if 'candidate_report' in data:
                report=dict(data['candidate_report'],target_id=req.target_id,stage=req.stage,
                            scene_version=version,scheduled_start_s=scheduled,computed_at_s=self.now(),
                            acquired_at_s=seconds(acquired_at),comparison_only=refresh_candidate)
                self.candidate_pub.publish(String(data=json.dumps(report,allow_nan=False)))
                self.get_logger().info(f"grasp candidates: {report.get('selected_candidate_id') or 'none'}; {report['scope']}")
                rejected = [item for item in report.get('candidates', []) if not item.get('feasible', False) and item.get('reason')]
                if rejected:
                    from deux_task_core.grasp_candidates import failure_summary
                    self.get_logger().warning(
                        f"grasp candidate rejection target={req.target_id} IK={data['ik_backend']}:\n"
                        + (f"{len(rejected)} rejected time/grip entries; see diagnostics/GUI" if report.get('time_search') else failure_summary(rejected)))
                    # Persist the inputs and wrapped causes for diagnosis after
                    # the volatile ROS report has gone away. No preview paths.
                    detail = {key: value for key, value in report.items()
                              if key not in ('candidates', 'ee_paths')}
                    detail.update(candidates=rejected, start_joints=start.tolist(),
                                  object_pose=target_pose.tolist(), velocity=motion[0], lead_s=motion[1])
                    self.get_logger().warning('grasp candidate diagnostics: ' + json.dumps(detail, allow_nan=False))
            if timed_approach and not in_time:
                if research_attempt>=self.planning_latency.config['max_research_attempts']:
                    raise ValueError('planning/dispatch budget exhausted after bounded future-time search')
                research_attempt+=1
                self.get_logger().warning('Reserved start expired; searching later interception times with measured compute budget')
                reservation=self.planning_latency.diagnostics()
                scheduled=self.planning_latency.reserve_start(self.now())
                refresh_inputs();timing=None;refresh_candidate=True
                work_wall=time.monotonic();work_clock=self.now()
                self.work=self.pool.submit(compute,c,version,req.stage,start,target_pose,None,motion,'',None,covariance)
                continue
            if data.get('planning_error'):raise ValueError(data['planning_error'])
            if reuse_path:
                # Retain the selected path, original acquisition and absolute start.
                # Only a materially different observation causes another search.
                check_wall=time.monotonic()
                latest_obs=deepcopy(self.observation)
                stamp=seconds(latest_obs.objects.header.stamp) if latest_obs is not None else -1.
                fresh=(latest_obs is not None and latest_obs.objects_valid and
                       seconds(acquired_at)<=stamp<=self.now() and
                       self.now()-stamp<=c['max_object_age'] and
                       any(item.track_id==req.target_id for item in latest_obs.objects.detections))
                if fresh:
                    latest_obj=self.target(req.target_id,latest_obs)
                    if not latest_obj.pose_filtered or not latest_obj.velocity_valid:
                        raise ValueError('filtered motion required for preflight')
                    v=latest_obj.velocity
                    latest_velocity=conveyor_velocity([v.linear.x,v.linear.y,v.linear.z,v.angular.z],c)
                    from deux_task_core.plan_preflight import compare_prediction
                    check=compare_prediction(c,target_pose,velocity,seconds(acquired_at),
                        pose_matrix(latest_obj.pose.pose),latest_velocity,stamp,
                        seconds(acquired_at)+data['timing']['grasp_start_lead_s']+sum(data['timing']['phase_durations']))
                else:
                    raise ValueError('fresh target required for path preflight')
                if max(abs(self.measured()-start))>c['start_tolerance']:
                    raise ValueError('robot moved during candidate search')
                if self.config_version!=version or self.now()<clock:
                    raise ValueError('scene/clock changed during preflight')
                check.update(original_acquired_at_s=seconds(acquired_at),checked_at_s=self.now(),
                             scheduled_start_s=scheduled,ik_recomputed=False)
                self.planning_latency.observe('revalidation',time.monotonic()-check_wall)
                self.candidate_pub.publish(String(data=json.dumps(dict(data['candidate_report'],
                    scope='validated path preflight; no repeated IK',preflight=check,
                    target_id=req.target_id,stage=req.stage,comparison_only=False,
                    acquired_at_s=seconds(acquired_at),scheduled_start_s=scheduled,
                    computed_at_s=self.now(),scene_version=version),allow_nan=False)))
                self.get_logger().info('Path preflight: '+json.dumps(check))
                if check['reuse']:break
                if research_attempt>=self.planning_latency.config['max_research_attempts']:
                    raise ValueError('observation changed after bounded future-time search')
                research_attempt+=1
                reservation=self.planning_latency.diagnostics()
                scheduled=self.planning_latency.reserve_start(self.now())
                refresh_inputs();timing=None;refresh_candidate=True
                work_wall=time.monotonic();work_clock=self.now()
                self.work=self.pool.submit(compute,c,version,req.stage,start,target_pose,None,motion,'',None,covariance)
                continue
            if not refresh_candidate:break
            # Refresh perception before executing the candidate. Timed mode
            # keeps the reserved start; the legacy mode chooses a new start.
            if data.get('timing'):
                timing=dict(data['timing'])
                previous_acquired=seconds(acquired_at)
            # A timed search already reserved computation AND dispatch. Preserve
            # that start and the absolute arrival; moving only the start squeezes
            # the approach duration and invalidates an otherwise feasible path.
            if not timed_approach:scheduled=self.now()+c['conveyor_tracking']['start_lead']
            refresh_inputs()
            if timing is not None:
                shift=previous_acquired-seconds(acquired_at)
                timing=dict(timing,arrival_lead_s=timing['arrival_lead_s']+shift,grasp_start_lead_s=timing['grasp_start_lead_s']+shift)
            work_wall=time.monotonic();work_clock=self.now()
            self.work=self.pool.submit(compute,c,version,req.stage,start,target_pose,None,motion,data['candidate_id'],timing,covariance)
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
        if data.get('timing'):
            plan.grasp_start_at=ros_time(seconds(acquired_at)+data['timing']['grasp_start_lead_s'])
            plan.grasp_phase_durations=data['timing']['phase_durations']
        elif timing is not None and req.stage=='TrackGrasp':
            plan.grasp_start_at=req.grasp_start_at
            plan.grasp_phase_durations=list(req.grasp_phase_durations)
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
        if timed_approach and time_settings(c).get('prefetch_grasp',False) and hasattr(self,'grasp_prefetch'):
            self.grasp_prefetch.arm(plan)
        return result

    def destroy_node(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    run(MotionPlannerNode, args)
