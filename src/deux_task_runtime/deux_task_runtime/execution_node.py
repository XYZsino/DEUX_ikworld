from deux_task_core.planning import requires_object_pose
from copy import deepcopy
import time
import numpy as np
from deux_task_core.placement import LOADED_STAGES
from std_msgs.msg import String
from deux_task_msgs.action import ExecuteMotion
from .arm_adapter import ArmAdapter
from .hand_adapter import HandAdapter
from deux_task_core.model import JOINT_NAMES, ARM_NAMES, HAND_NAMES, RobotModel
from .common import (
    RuntimeNode,
    ExclusiveAction,
    seconds,
    ros_time,
    pose_matrix,
    pause,
    wait,
    run,
    current,
)
from .trajectory import arrays, sample


class ExecutionNode(RuntimeNode):
    def __init__(self, **kwargs):
        super().__init__("execution_node", observe_state=True, observe_barcode=True, observe_status=True, **kwargs)
        self.declare_parameter("dry_run", True)
        if self.get_parameter("dry_run").value:
            self.subscribe_joints()
        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("mapping_file", "")
        self.declare_parameter("driver_action", "/arm/deux_arm_controller/follow_joint_trajectory")
        self.arm = ArmAdapter(self)
        self.hand = HandAdapter(self)
        self.record = self.create_publisher(String, "/task/execution_record", 10)
        self.action = ExclusiveAction(
            self, ExecuteMotion, "/task/execute_motion", self.execute, self.validate
        )
        self.completed = set()

    def measured(self, obs=None):
        if self.get_parameter("dry_run").value:
            return super().measured(obs)
        # Use raw feedback directly for control; do not wait for its round trip
        # through StateObservation. Perception/EE guards still use that stream.
        values = dict(zip(ARM_NAMES, self.arm.measured_arm()))
        values.update(zip(HAND_NAMES, self.hand.measured_hand()))
        return np.array([values[name] for name in JOINT_NAMES])

    def barcode_guard(self, plan):
        if self.config.get('barcode_fixed_wrist'):
            from deux_task_core.reader_alignment import check_posture
            if not hasattr(self,'reader_robot'):self.reader_robot=RobotModel(self.config['robot'])
            check_posture(self.reader_robot,self.measured(),self.config)
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
        if expected.barcode_track_id and observed.barcode_track_id != expected.barcode_track_id:
            raise ValueError("barcode track identity changed")
        self.check_target_rotation(observed.pose, expected.pose)
        if np.linalg.norm(pose_matrix(observed.pose)[:3,3]-pose_matrix(expected.pose)[:3,3]) > self.config['max_target_drift']:
            raise ValueError('barcode moved; replan')

    def frozen_grasp(self, plan):
        return not self.config.get('moving_source',False) and self.config.get('perception_policy')=='head_rgbd' and plan.stage in ('Approach','Descend','Close')

    def conveyor_guard(self, plan):
        from deux_task_core.conveyor import forecast,inside,uses_workspace_box
        if not self.config.get('moving_source',False) or not plan.follows_conveyor:
            raise ValueError('moving source requires conveyor trajectory')
        cfg=self.config['conveyor_tracking']
        obs=self.observation
        age=self.now()-seconds(obs.objects.header.stamp) if obs is not None else float('inf')
        tracking=(plan.stage=='TrackGrasp' and len(plan.phase_end_times)==3 and
                  -cfg.get('start_lead',0.)<=self.now()-seconds(plan.trajectory.header.stamp)<(plan.phase_end_times[-1]+self.config['settle_timeout'] if cfg.get('occluded_grasp_enabled',False) else plan.phase_end_times[0]))
        if plan.stage=='Approach' and cfg.get('occluded_grasp_enabled',False):tracking=True
        prediction=age>self.config['max_object_age'] or obs is None or not obs.objects_valid
        grace=cfg.get('occlusion_prediction_max_age',self.config['max_object_age'])
        if plan.stage=='Approach':grace=min(grace,cfg.get('approach_prediction_max_age',self.config['max_object_age']))
        if prediction and tracking:
            cached=getattr(self,'conveyor_last_observation',None)
            # An asynchronously delivered observation may already be older than
            # the normal freshness limit, yet newer than our last control tick.
            # Keep the newest original acquisition, never substitute an older one.
            if cached is not None and (obs is None or not obs.objects_valid or
                    seconds(cached.objects.header.stamp)>seconds(obs.objects.header.stamp)):
                obs=cached
        obj=self.target(plan.target_id,obs=obs,max_age=grace if prediction and tracking else None)
        if not prediction:self.conveyor_last_observation=deepcopy(obs)
        if prediction:
            if not tracking or obs is None:raise ValueError('fresh pose required outside permitted prediction phases')
            age=self.now()-seconds(obs.objects.header.stamp)
            from deux_task_core.conveyor import validate_occlusion_prediction
            radius=validate_occlusion_prediction(obj,plan,age,self.now(),self.config)
            hands=[JOINT_NAMES.index(name) for name in HAND_NAMES]
            if cfg.get('occluded_grasp_enabled',False) and plan.stage!='Approach':
                # A stale perception stream alone cannot authorize a blind reach.
                # Prediction is limited to a hand already near the expected target.
                measured=self.measured()
                if not hasattr(self,'conveyor_robot'):self.conveyor_robot=RobotModel(self.config['robot'])
                ee=self.conveyor_robot.fk(measured)[self.conveyor_robot.ee['right']]
                v=plan.target_velocity
                expected=forecast(pose_matrix(plan.target_pose),[v.linear.x,v.linear.y,v.linear.z,v.angular.z],self.now()-seconds(plan.target_acquired_at))
                if np.linalg.norm(ee[:3,3]-expected[:3,3])>cfg['occlusion_hand_distance_m']:
                    raise ValueError('prediction requires hand near target')
            elif not cfg.get('occluded_grasp_enabled',False) and np.max(abs(self.measured()[hands]-self.config.get('hand_pregrasp',self.config['hand_open'])))>.05:
                raise ValueError('open hand required for occlusion prediction')
            self.conveyor_last_observation=deepcopy(obs)
            self.record.publish(String(data=f'{plan.plan_id}: bounded conveyor prediction; acquisition={seconds(obs.objects.header.stamp):.3f}; age={age:.3f}; 2sigma_m={radius:.4f}'))
        if not obj.pose_filtered or not obj.velocity_valid:raise ValueError('fresh filtered target required during tracking')
        actual=pose_matrix(obj.pose.pose)
        if uses_workspace_box(self.config):
            region=cfg['acquisition_region'] if plan.stage=='Approach' else cfg['grasp_region']
            if not inside(actual[:3,3],region):raise ValueError('target left permitted tracking region')
        if plan.stage=='Approach':
            if not plan.trajectory.points:raise ValueError('missing interception trajectory')
            remaining=max(0.,seconds(plan.trajectory.header.stamp)+seconds(plan.trajectory.points[-1].time_from_start)-seconds(obs.objects.header.stamp))
            v=plan.target_velocity
            arrival=forecast(actual,[v.linear.x,v.linear.y,v.linear.z,v.angular.z],remaining)
            if uses_workspace_box(self.config) and not inside(arrival[:3,3],cfg['grasp_region'],cfg['boundary_margin']):raise ValueError('forecast interception left grasp region')
        v=plan.target_velocity
        dt=seconds(obs.objects.header.stamp)-seconds(plan.target_acquired_at)
        if not 0<=dt<=cfg['max_plan_horizon']:raise ValueError('conveyor forecast expired')
        expected=forecast(pose_matrix(plan.target_pose),[v.linear.x,v.linear.y,v.linear.z,v.angular.z],dt)
        if np.linalg.norm(actual[:3,3]-expected[:3,3])>self.config['max_target_drift']:
            raise ValueError('conveyor innovation requires replan')
        from scipy.spatial.transform import Rotation
        if Rotation.from_matrix(actual[:3,:3]@expected[:3,:3].T).magnitude()>self.config.get('max_target_angle_rad',.15):
            raise ValueError('conveyor orientation requires replan')

    def grasp_age_guard(self, plan):
        if self.frozen_grasp(plan) and not current(plan.target_acquired_at,self.now(),self.config['grasp_pose_max_age']):
            raise ValueError('original grasp pose expired')

    def validate(self, request):
        self.check_model_version()
        p = request.plan
        from deux_task_core.ik_backends import backend_name
        if (p.ik_backend or 'scipy') != backend_name(self.config):
            raise ValueError('plan IK backend differs from installation')
        if p.follows_conveyor:self.conveyor_guard(p)
        elif self.config.get('moving_source',False) and p.stage in ('Approach','Descend','Close','TrackGrasp'):
            raise ValueError('static grasp plan rejected for moving source')
        self.grasp_age_guard(p)
        if p.stage == "AlignBarcode": self.barcode_guard(p)
        q = self.measured()
        from deux_task_core.placement import check_start, enabled, PLACEMENT_STAGES
        if enabled(self.config) and p.stage in PLACEMENT_STAGES:
            check_start(RobotModel(self.config['robot']), self.config, p.stage, q)
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
        if p.stage=='TrackGrasp':
            phases=np.asarray(p.phase_end_times)
            if phases.shape!=(3,) or not np.isfinite(phases).all() or phases[0]<0 or np.any(np.diff(phases)<=0) or abs(phases[-1]-t[-1])>1e-6:
                raise ValueError("invalid interception phase timing")
        if np.max(abs(path[0] - q)) > self.config["start_tolerance"]:
            raise ValueError("start mismatch")
        if requires_object_pose(self.config, p.stage) and not self.frozen_grasp(p) and not p.follows_conveyor:
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
        from .trajectory import validate_limits
        validate_limits(plan.trajectory, JOINT_NAMES, robot)
        if plan.grasp_phase_durations and plan.stage=='TrackGrasp':
            phases=np.asarray(plan.grasp_phase_durations)
            if (phases.shape!=(3,) or not np.isfinite(phases).all() or phases[0]<0
                    or np.any(phases[1:]<=0) or len(plan.phase_end_times)!=3
                    or not np.allclose(np.cumsum(phases),plan.phase_end_times,atol=1e-6)
                    or abs(seconds(plan.trajectory.header.stamp)-seconds(plan.grasp_start_at))>1e-6
                    or abs(t[-1]-phases.sum())>1e-6):
                raise ValueError('timed grasp schedule mismatch')
        if np.any(np.diff(t) > c["sample_period"] * 1.01):
            raise ValueError("trajectory sampling too sparse")
        attached = (
            c["attached_radius"]
            if plan.stage in LOADED_STAGES
            else 0.0
        )
        for row in q:
            if plan.stage=='AlignBarcode' and c.get('barcode_fixed_wrist'):
                from deux_task_core.reader_alignment import check_posture
                check_posture(robot,row,c,command=True)
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
        self.arm.validate()
        hand_indices = [JOINT_NAMES.index(n) for n in HAND_NAMES]
        hand_q = q[:, hand_indices]
        self.hand.validate(t, hand_q)
        pending = handle = None
        hand_started = hand_completed = False
        start = seconds(plan.trajectory.header.stamp) if plan.follows_conveyor else self.now() + (c.get("lift_start_lead",.5) if plan.stage=="Lift" else .5)
        if start-self.now()<.1:raise ValueError('scheduled tracking start missed')
        deadline = start + t[-1] + c["settle_timeout"]
        try:
            tr = deepcopy(plan.trajectory)
            indices = [JOINT_NAMES.index(n) for n in ARM_NAMES]
            tr.joint_names = list(ARM_NAMES)
            tr.header.stamp = ros_time(start)
            for point in tr.points:
                point.positions = [point.positions[i] for i in indices]
            pending = self.arm.send(tr)
            handle = await wait(self, pending, max(0.01, start - self.now() - 0.1), goal)
            if not handle.accepted:
                raise ValueError("arm driver rejected")
            if self.now() >= start:
                raise TimeoutError("arm acceptance missed synchronized start")
            result = self.arm.result_for(handle)
            wall = time.monotonic()
            previous = self.now()
            while True:
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
                if plan.follows_conveyor:self.conveyor_guard(plan)
                if (
                    np.max(abs(measured - sample(t, q, max(0, now - start))))
                    > c["tracking_tolerance"]
                ):
                    raise ValueError("tracking error")
                self.grasp_age_guard(plan)
                if plan.stage in ("Approach", "Descend") and not self.frozen_grasp(plan) and not plan.follows_conveyor:
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
                arm_completed = self.arm.check_result(result)
                elapsed = now - start
                if elapsed >= 0 and not hand_completed:
                    hand_started = True
                    self.hand.command(sample(t, hand_q, elapsed))
                    hand_completed = (elapsed >= t[-1] and
                        np.max(abs(measured[hand_indices] - hand_q[-1])) <= c["start_tolerance"])
                if arm_completed and hand_completed:
                    break
                goal.publish_feedback(
                    ExecuteMotion.Feedback(
                        progress=float(np.clip((now - start) / t[-1], 0, 1)),
                        status=('Track','Descend','Close')[min(2,int(np.searchsorted(plan.phase_end_times,max(0,now-start),side='right')))] if plan.stage=='TrackGrasp' else plan.stage,
                    )
                )
                await pause(self)
            if np.max(abs(self.measured() - q[-1])) > c["start_tolerance"]:
                raise ValueError("final measured position mismatch")
            return ExecuteMotion.Result(
                success=True,
                reason="measured motion complete; task outcome requires sensor evidence",
            )
        finally:
            # Do not replace a successful close's preload with stalled feedback.
            # A cancellation before hand dispatch must not issue a new hand command.
            try:
                if hand_started and not hand_completed:
                    self.hand.hold_measured()
            finally:
                await self.arm.cancel(pending, handle)


def main(args=None):
    run(ExecutionNode, args)
