"""Observation-based stage goals and time-parametrized monotone Hermite paths."""

import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation, Slerp
from .model import RobotModel, JOINT_NAMES, HAND_NAMES, vector, pose
from .placement import enabled as placement_enabled, LOADED_STAGES

STAGES = (
    "Prepare",
    "ReadyPose",
    "ObserveScene",
    "TrackGrasp",
    "Approach",
    "Descend",
    "Close",
    "Lift",
    "Scan",
    "ScanPose",
    "AlignBarcode",
    "ClearReader",
    "PlaceApproach",
    "PlaceRetreat",
    "Place",
    "Release",
    "Retreat",
    "Recover",
)


def requires_object_pose(config, stage):
    if stage in ("Prepare", "ReadyPose"):
        return False
    if config.get("perception_policy") == "head_rgbd":
        return stage in ("Approach", "Descend", "Close", "TrackGrasp") or (stage == "Place" and not placement_enabled(config))
    return True


def grasp_cache_max_age(config):
    """Keep a frozen pose across manual review only for a confirmed stationary source."""
    disabled = config.get('stationary_grasp_pose_expiry_disabled', False)
    if type(disabled) is not bool:
        raise ValueError('stationary_grasp_pose_expiry_disabled must be boolean')
    if disabled:
        if (config.get('moving_source', False)
                or config.get('stationary_source_confirmed') is not True
                or config.get('perception_policy') != 'head_rgbd'):
            raise ValueError('unlimited grasp cache requires a confirmed stationary head_rgbd source')
        return float('inf')
    return config.get('grasp_pose_max_age', 0)


def validate_scene(config):
    grasp_cache_max_age(config)
    vector(config.get('barcode_aim_offset_base_m', [0., 0., 0.]), 3)
    visibility = config.get('stationary_grasp_visibility_check_enabled', True)
    if type(visibility) is not bool:
        raise ValueError('stationary_grasp_visibility_check_enabled must be boolean')
    if not visibility and (config.get('moving_source',False)
            or config.get('stationary_source_confirmed') is not True
            or config.get('perception_policy') != 'head_rgbd'):
        raise ValueError('disabled grasp visibility requires confirmed stationary head_rgbd source')
    if type(config.get('motion_review_required',False)) is not bool:
        raise ValueError('motion_review_required must be boolean')
    review_timeout=config.get('motion_review_timeout_s',300.)
    if not np.isfinite(review_timeout) or not 1 <= review_timeout <= 600:
        raise ValueError('motion_review_timeout_s must be 1..600')
    if type(config.get('right_hand_disabled',False)) is not bool:
        raise ValueError('right_hand_disabled must be boolean')
    if config.get('right_hand_disabled',False) and not config.get('hand_command_only',False):
        raise ValueError('right-hand-disabled requires hand_command_only')
    if type(config.get('skip_barcode_completion',False)) is not bool:
        raise ValueError('skip_barcode_completion must be boolean')
    if type(config.get('hand_command_only',False)) is not bool:
        raise ValueError('hand_command_only must be boolean')
    if type(config.get('right_hand_preload',False)) is not bool:
        raise ValueError('right_hand_preload must be boolean')
    for key in ('hand_close_duration_s','hand_open_duration_s'):
        if key in config and (not np.isfinite(config[key]) or config[key] <= 0):
            raise ValueError('positive finite hand duration required')
    from .ik_backends import backend_name
    backend_name(config)
    from .grasp_candidates import validate as validate_candidates
    validate_candidates(config)
    if config.get('conveyor_tracking',{}).get('workspace_policy','box') not in ('box','ik'):
        raise ValueError('invalid conveyor workspace_policy')
    selection=config.get('target_selection_policy','legacy')
    if selection not in ('legacy','rgb_roi_3d_bbox'):
        raise ValueError('invalid target_selection_policy')
    if 'selection_roi_vertical_fraction' in config:
        y=vector(config['selection_roi_vertical_fraction'],2)
        if not 0<=y[0]<y[1]<=1:raise ValueError('invalid selection ROI vertical fraction')
    if selection=='rgb_roi_3d_bbox':
        region=config['conveyor_tracking']['acquisition_region']
        if np.any(vector(region['min'],3)>=vector(region['max'],3)):
            raise ValueError('invalid RGB selection projection region')
    if config.get('placement'):
        from .placement import validate
        validate(config)
    if config.get("barcode_fixed_wrist"):
        from .reader_alignment import validate_settings
        validate_settings(config["barcode_fixed_wrist"])
    angles=np.asarray(config.get('barcode_view_angles_deg',[[0.,0.]]),float)
    if angles.ndim!=2 or angles.shape[1]!=2 or not 1<=len(angles)<=16 or not np.isfinite(angles).all() or np.any(np.abs(angles)>60.):
        raise ValueError('invalid barcode_view_angles_deg')
    rolls=np.asarray(config.get('barcode_reader_roll_candidates',[0.]),float)
    if rolls.ndim!=1 or not 1<=len(rolls)<=16 or not np.isfinite(rolls).all() or np.any(np.abs(rolls)>np.pi):
        raise ValueError('invalid barcode_reader_roll_candidates')
    for key, default, lower, upper in (
        ('lift_cartesian_step',.01,.01,.05),
        ('lift_minimum_segment_duration',.3,.02,.3),
        ('descend_minimum_segment_duration',.3,.02,.3),
        ('lift_start_lead',.5,.15,.5),
    ):
        value=config.get(key,default)
        if not np.isfinite(value) or not lower<=value<=upper:
            raise ValueError(f'invalid {key}')
    approach_accel=config.get('approach_arm_acceleration_scale',1.)
    if not np.isfinite(approach_accel) or not 0 < approach_accel <= 1:
        raise ValueError('approach_arm_acceleration_scale must be in (0, 1]')
    scan_scale=config.get('scan_motion_scale',1.)
    if not np.isfinite(scan_scale) or not 0 < scan_scale <= 1:
        raise ValueError('scan_motion_scale must be in (0, 1]')
    if config.get("perception_policy") == "head_rgbd":
        vector(config["scan_right_joints"], 7)
        vector(config["observe_right_joints"], 7)
        if config.get('moving_source',False):
            tracking=config['conveyor_tracking']
            if not config['max_object_age']<=tracking['occlusion_prediction_max_age']<=(3. if tracking.get('occluded_grasp_enabled',False) else 1.):raise ValueError('invalid bounded tracking prediction age')
            if not config['max_object_age']<=tracking.get('approach_prediction_max_age',config['max_object_age'])<=tracking['occlusion_prediction_max_age']:raise ValueError('invalid approach prediction age')
            if tracking.get('occluded_grasp_enabled',False):
                for key in ('belt_velocity_std_m_s','belt_yaw_rate_std_rad_s','prediction_yaw_acceleration_std','belt_velocity_tolerance_m_s','belt_yaw_rate_tolerance_rad_s','occlusion_hand_distance_m'):
                    if not np.isfinite(tracking[key]) or tracking[key]<=0:raise ValueError('invalid belt prediction uncertainty')
            if tracking.get('intercept_wait_s',0.):
                for key in ('intercept_wait_s','intercept_trigger_distance_m','intercept_height_tolerance_m','intercept_wait_timeout_s'):
                    if not np.isfinite(tracking[key]) or tracking[key]<=0:raise ValueError('invalid rendezvous limits')
            if not 0<=tracking.get('close_descent_overlap',0.)<=.5:raise ValueError('invalid descent closure overlap')
            if not .5<=tracking.get('grasp_planning_ratio',.98)<=.98:raise ValueError('invalid grasp planning dynamics margin')
            if not 0<tracking['prediction_acceleration_std']<=1.:raise ValueError('invalid prediction process noise')
            if np.linalg.norm(vector(tracking['nominal_velocity'],4)[:3])>tracking['max_speed']:raise ValueError('conveyor prior exceeds speed limit')
            for key in ('max_speed','start_lead','max_plan_horizon','boundary_margin','velocity_stability_m_s'):
                if not np.isfinite(tracking[key]) or tracking[key]<=0:raise ValueError('invalid conveyor tracking limits')
            for key in ('grasp_region','acquisition_region'):
                if np.any(vector(tracking[key]['min'],3)>=vector(tracking[key]['max'],3)):raise ValueError('invalid tracking region')
            if not isinstance(tracking['min_motion_observations'],int) or tracking['min_motion_observations']<3:raise ValueError('insufficient velocity observations')
            if abs(np.linalg.norm(vector(tracking['flow_direction'],3))-1)>1e-6:raise ValueError('flow direction must be a unit vector')
            for stage in ('Approach','Track','Descend','Close'):
                if not np.isfinite(tracking['durations'][stage]) or (tracking['durations'][stage]<0 if stage=='Track' else tracking['durations'][stage]<=0):raise ValueError('invalid conveyor stage duration')
        elif config.get("stationary_source_confirmed") is not True or not 0 < config.get("grasp_pose_max_age", 0) <= 20:
            raise ValueError("bounded grasp pose cache requires a confirmed stationary source")
    if not isinstance(config.get("prepare_before_first_task", False), bool):
        raise ValueError("prepare_before_first_task must be boolean")
    if config["schema"] != "deux.scene.v1" or not config["base_frame"]:
        raise ValueError("invalid scene schema/frame")
    if not config["model_id"]:
        raise ValueError("explicit object model_id required")
    if config["robot"]["base_frame"] != config["base_frame"]:
        raise ValueError("robot/scene frame mismatch")
    for key in (
        "grasp_object_from_right_ee",
        "scan_right_pose",
        "scan_left_pose",
        "retreat_left_pose",
        "destination_pose",
    ):
        pose(config[key])
    if not config.get("synthetic", False):
        pose(config["left_ee_from_reader"])
        if not config["object_from_labels"] and config.get("perception_policy")!="head_rgbd":
            raise ValueError("calibrated label poses required")
        for label in config["object_from_labels"]:
            pose(label)
        if not np.isfinite(config["read_distance"]) or config["read_distance"] <= 0:
            raise ValueError("read distance required")
    for key in ("hand_open", "hand_closed"):
        vector(config[key], 14)
    if 'hand_pregrasp' in config:
        pre=vector(config['hand_pregrasp'],14)
        if np.any(pre<np.minimum(config['hand_open'],config['hand_closed'])) or np.any(pre>np.maximum(config['hand_open'],config['hand_closed'])):
            raise ValueError('pregrasp must lie between open and closed')
    vector(config["home_joints"], 28)
    for waypoint in config.get("prepare_joint_waypoints", []):
        vector(waypoint, 28)
    for key in (
        "approach_height",
        "lift_height",
        "max_target_drift",
        "max_observation_age",
        "max_joint_age",
        "max_object_age",
        "max_plan_age",
        "sample_period",
        "collision_margin",
        "start_tolerance",
        "tracking_tolerance",
        "settle_timeout",
        "scan_timeout",
        "verification_timeout",
    ):
        if not np.isfinite(config[key]) or config[key] <= 0:
            raise ValueError(f"positive {key} required")
    # Zero disables the optional carried-object collision envelope.
    if not np.isfinite(config["attached_radius"]) or config["attached_radius"] < 0:
        raise ValueError("nonnegative attached_radius required")
    if 'descend_distance_m' in config:
        distance=config['descend_distance_m']
        if not np.isfinite(distance) or distance <= 0:
            raise ValueError('positive descend_distance_m required')
        if config.get('moving_source',False):
            raise ValueError('descend_distance_m requires a stationary source')
    final_tolerance = config.get("final_tolerance", config["start_tolerance"])
    if not np.isfinite(final_tolerance) or final_tolerance <= 0:
        raise ValueError("positive final_tolerance required")
    if config["sample_period"] > 0.05:
        raise ValueError("sample_period must be <= 50ms")
    for name in ("workspace", "destination_bounds"):
        box = config[name]
        if np.any(vector(box["min"], 3) >= vector(box["max"], 3)):
            raise ValueError("invalid bounds")
    if not isinstance(config["obstacles"], list):
        raise ValueError("explicit obstacles list required")
    for box in config["obstacles"]:
        if np.any(vector(box["min"], 3) >= vector(box["max"], 3)):
            raise ValueError("invalid obstacle")
    if not np.isclose(config["source_box_height"], 0.07):
        raise ValueError("source box height contract is 0.07m")
    if config.get("safe_joint_waypoint") is not None:
        vector(config["safe_joint_waypoint"], 28)
    if (
        not np.isfinite(config.get("max_target_angle_rad", 0.15))
        or config.get("max_target_angle_rad", 0.15) <= 0
    ):
        raise ValueError("positive max_target_angle_rad required")
    scale = config.get('simulation_wall_timeout_scale', 1.)
    if not np.isfinite(scale) or not 1 <= scale <= 20 or (scale != 1 and not config.get('synthetic', False)):
        raise ValueError('simulation wall timeout scale requires synthetic scene, range 1..20')
    # Checking configuration never certifies calibration on actual hardware.
    return config


def scene_version(config):
    payload = json.dumps(config, sort_keys=True, allow_nan=False).encode()
    robot_bytes = Path(config["robot"]["urdf_path"]).read_bytes()
    return hashlib.sha256(payload + b"\0" + robot_bytes).hexdigest()


def hermite(knots, velocity, acceleration, dt, minimum_segment_duration=0.3, corner_blend=0.):
    if not np.isfinite(corner_blend) or not 0 <= corner_blend <= 1:
        raise ValueError('corner_blend must be in [0, 1]')
    knots = np.asarray(knots, dtype=float)
    durations = np.maximum(
        minimum_segment_duration, 2 * np.max(np.abs(np.diff(knots, axis=0)) / velocity, axis=1)
    )
    for _ in range(12):
        durations = np.ceil(durations / dt) * dt
        slopes = np.diff(knots, axis=0) / durations[:, None]
        tangents = np.zeros_like(knots)
        for i in range(1, len(knots) - 1):
            same = slopes[i - 1] * slopes[i] > 0
            tangents[i, same] = 2 / (1 / slopes[i - 1, same] + 1 / slopes[i, same])
            # Optional fixed-route smoothing: move each reversing joint's
            # zero crossing away from the common knot, avoiding a full-arm stop.
            reversing = ~same
            tangents[i, reversing] = corner_blend * .5 * (slopes[i-1, reversing] + slopes[i, reversing])
        times = np.r_[0, np.cumsum(durations)]
        spline = CubicHermiteSpline(times, knots, tangents)
        sample = np.linspace(0, times[-1], int(round(times[-1] / dt)) + 1)
        # Dense timing check includes both sides of each knot.
        dense = np.unique(np.r_[sample, times, np.maximum(times - 1e-9, 0)])
        factor = max(
            np.max(np.abs(spline(dense, 1)) / velocity),
            np.sqrt(np.max(np.abs(spline(dense, 2)) / acceleration)),
        )
        if factor <= 1.000001:
            return sample, spline(sample)
        durations *= factor * 1.02
    raise ValueError("trajectory timing failed")


def continuous_joint_path(knots, velocity, acceleration, dt):
    """C2 joint curve, arc-length timing, cosine start/stop ramps.

    Internal waypoints are transit points. Only the two endpoints stop.
    Every returned path still undergoes the caller's joint/geometry checks.
    """
    from scipy.interpolate import CubicSpline
    knots=np.asarray(knots,float)
    keep=np.r_[True,np.linalg.norm(np.diff(knots,axis=0),axis=1)>1e-9]
    knots=knots[keep]
    if len(knots)<2:
        return np.array([0.,dt]),np.repeat(knots[:1],2,axis=0)
    chord=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(knots,axis=0),axis=1))]
    spline=CubicSpline(chord,knots,bc_type='natural')
    u=np.linspace(0,chord[-1],8001)
    curve=spline(u)
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(curve,axis=0),axis=1))]
    duration=max(.3,2*np.max(np.sum(abs(np.diff(knots,axis=0)),axis=0)/velocity))
    for _ in range(12):
        duration=np.ceil(duration/dt)*dt
        t=np.linspace(0,duration,int(round(duration/dt))+1)
        x=t/duration; ramp=.2; v=1/(1-ramp)
        def start(z):return v*(z/2-ramp/(2*np.pi)*np.sin(np.pi*z/ramp))
        fraction=np.where(x<ramp,start(x),np.where(x>1-ramp,1-start(1-x),v*(x-ramp/2)))
        path=spline(np.interp(fraction*arc[-1],arc,u))
        speed=np.diff(path,axis=0)/dt
        acc=np.diff(speed,axis=0)/dt
        factor=max(np.max(abs(speed)/velocity),np.sqrt(np.max(abs(acc)/acceleration)))
        if factor<=.98:
            return t,path
        duration*=max(1.01,factor/.97)
    raise ValueError('continuous trajectory timing failed')


def fixed_hand_path(knots, velocity, acceleration, dt, duration):
    """Exact-duration zero-end-velocity hand phase; never silently retime."""
    knots=np.asarray(knots,float)
    if len(knots)!=2:
        raise ValueError('fixed hand duration requires a direct hand phase')
    delta=knots[1]-knots[0]
    if np.any(1.5*np.abs(delta)/duration > velocity*1.000001):
        raise ValueError('requested hand duration exceeds joint velocity limits')
    if np.any(6*np.abs(delta)/duration**2 > acceleration*1.000001):
        raise ValueError('requested hand duration exceeds joint acceleration limits')
    times=np.linspace(0,duration,int(np.ceil(duration/dt))+1)
    u=times/duration
    return times,knots[0]+(3*u*u-2*u*u*u)[:,None]*delta


class Planner:
    def __init__(self, config):
        self.config = validate_scene(config)
        self.robot = RobotModel(config["robot"])
        from .ik_backends import attach_backend
        attach_backend(self.robot,config)

    def plan(self, stage, start, object_pose, canceled=lambda: False, barcode_pose=None):
        override = self.config.get('placement', {}).get('descent_ik_rotation_tolerance_rad') if stage == 'Place' else None
        if override is None or self.config.get('placement', {}).get('fixed_joint_motion', False):
            return Planner._plan(self, stage, start, object_pose, canceled, barcode_pose)
        original = self.robot.ik_rotation_tolerance_rad
        try:
            self.robot.ik_rotation_tolerance_rad = float(override)
            return Planner._plan(self, stage, start, object_pose, canceled, barcode_pose)
        finally:
            self.robot.ik_rotation_tolerance_rad = original

    def _plan(self, stage, start, object_pose, canceled=lambda: False, barcode_pose=None):
        c = self.config
        if stage not in STAGES:
            raise ValueError("unknown stage")
        if stage == 'Approach' and c.get('temporary_axis_grasp_trial', False):
            from .axis_grasp_trial import select
            return select(self, start, object_pose, canceled)
        q = vector(start, 28).copy()
        if c.get("hand_command_only",False):
            # Use explicit command postures for planning; raw feedback remains
            # available to runtime fault checks and arm/EE geometry.
            q[7:14]=c["hand_closed"][:7]
            a=np.asarray(c["hand_open"])[7:];b=np.asarray(c["hand_closed"])[7:]
            q[21:28]=b if np.linalg.norm(q[21:28]-b)<np.linalg.norm(q[21:28]-a) else a
        from .placement import check_start
        check_start(self.robot, c, stage, q)
        obj = pose(object_pose) if requires_object_pose(c, stage) else np.eye(4)
        if requires_object_pose(c, stage) and (np.any(obj[:3, 3] < c["workspace"]["min"]) or np.any(
            obj[:3, 3] > c["workspace"]["max"]
        )):
            raise ValueError("object outside workspace")
        fk = self.robot.fk(q)
        goal = q.copy()
        cartesian_target = None
        fixed_points = None
        fixed_placement = placement_enabled(c) and c['placement'].get('fixed_joint_motion', False)
        if fixed_placement and stage in ('PlaceApproach','Place','PlaceRetreat','Retreat'):
            from .placement import joint_points
            fixed_points=joint_points(c,stage)
        hand_indices = [JOINT_NAMES.index(n) for n in HAND_NAMES]
        if stage in LOADED_STAGES and stage not in ('Recover','ReadyPose'):
            # Preserve the configured closing setpoint across loaded stages,
            # rather than reducing grip preload to measured stalled positions.
            # Executor still checks this start against measured tolerance.
            q[hand_indices] = c['hand_closed']
            goal[hand_indices] = c['hand_closed']
        if c.get("right_hand_preload",False) and stage=="Release":
            q[21:28]=np.asarray(c["hand_closed"])[7:]
            goal[21:28]=q[21:28]
        if fixed_points is not None:
            goal[14:21] = vector(fixed_points[-1],7)
            if stage == 'Retreat':
                goal[:7] = vector(c['home_joints'],28)[:7]
        elif stage == "Prepare":
            goal = vector(c["home_joints"], 28).copy()
        elif stage == 'ReadyPose':
            # Preserve left-hand grip while returning the arms; open the right hand.
            # Keep the measured starting posture for a continuous planned path.
            for offset in (0,7):
                indices=hand_indices[offset:offset+7]
                closed=np.asarray(c['hand_closed'][offset:offset+7])
                opened=np.asarray(c['hand_open'][offset:offset+7])
                close_error=np.max(np.abs(q[indices]-closed))
                if (close_error <= c['start_tolerance']
                        and np.linalg.norm(q[indices]-closed) < np.linalg.norm(q[indices]-opened)):
                    q[indices]=closed
            goal=vector(c['home_joints'],28).copy()
            goal[hand_indices]=q[hand_indices]
            goal[21:28]=np.asarray(c['hand_open'])[7:14]
        elif stage in ("Approach", "Descend"):
            target = obj @ pose(c["grasp_object_from_right_ee"])
            if stage == "Approach":
                target[2, 3] += c["approach_height"]
            # Travel to the pre-grasp pose in joint space: a straight Cartesian
            # wrist rotation from home can cross an unreachable configuration.
            # Descend remains Cartesian and all paths retain collision checks.
            if stage == "Descend":
                if 'descend_distance_m' in c:
                    # Fixed base-Z drop from the measured approach endpoint.
                    target = fk[self.robot.ee['right']].copy()
                    target[2,3] -= c['descend_distance_m']
                    if c.get('temporary_axis_grasp_trial', False) or c.get('_axis_trial_hold_descent', False):
                        target[:3,:3] = fk[self.robot.ee['right']][:3,:3]
                cartesian_target = target
            goal = self.robot.ik("right", target, goal)
            goal[hand_indices] = c.get('hand_pregrasp',c["hand_open"])
        elif stage in ("Close", "Release"):
            if stage == 'Release' and placement_enabled(c):
                # The left hand keeps holding its scanner throughout placement.
                goal[21:28] = vector(c['hand_open'], 14)[7:14]
            else:
                goal[hand_indices] = c["hand_closed" if stage == "Close" else "hand_open"]
        elif stage == "Lift":
            target = fk[self.robot.ee["right"]].copy()
            target[2, 3] += c["lift_height"]
            cartesian_target = target
            goal = self.robot.ik("right", target, goal)
        elif stage == "ObserveScene":
            if c.get("perception_policy") != "head_rgbd":
                raise ValueError("ObserveScene requires head RGB-D policy")
            goal[14:21] = vector(c["observe_right_joints"], 7)
        elif stage == "ScanPose":
            if c.get("perception_policy") != "head_rgbd":
                raise ValueError("ScanPose requires head RGB-D policy")
            goal[14:21] = vector(c["scan_right_joints"], 7)
        elif stage == "AlignBarcode":
            if barcode_pose is None:
                raise ValueError('observed barcode surface required')
            if c.get('barcode_position_only', False):
                from .barcode_position import reader_look_at
                ee_from_reader=pose(c['left_ee_from_reader'])
                current_reader=fk[self.robot.ee['left']]@ee_from_reader
                target_reader=reader_look_at(current_reader,pose(barcode_pose)[:3,3],c['read_distance'],
                    c.get('barcode_aim_offset_base_m', [0., 0., 0.]), yaw_offset_deg=-30.)
                solver=getattr(self.robot,'ik_solver',None)
                if not hasattr(solver,'solve_reader_axis'):
                    raise ValueError('position-only barcode alignment requires PyRoKi reader-axis IK')
                goal=solver.solve_reader_axis('left',target_reader,ee_from_reader,goal)
                # Operator-requested extra wrist bend after reader-axis alignment.
                goal[5] += np.deg2rad(30.)
                if not self.robot.limits[5,0] <= goal[5] <= self.robot.limits[5,1]:
                    raise ValueError('left J6 +30deg scan bend exceeds joint limit')

            elif c.get('barcode_fixed_wrist'):
                from .reader_alignment import solve_fixed_wrist
                goal=solve_fixed_wrist(self.robot,q,barcode_pose,c,canceled)
                q[4:7]=c['barcode_fixed_wrist']['joint_values']
            else:
                candidates=[]
                for pitch,skew in c.get('barcode_view_angles_deg',[[0.,0.]]):
                    ray=np.array([np.tan(np.deg2rad(skew)),np.tan(np.deg2rad(pitch)),1.])
                    ray/=np.linalg.norm(ray)
                    x_axis=np.array([1.,0.,0.]);x_axis-=ray*np.dot(x_axis,ray);x_axis/=np.linalg.norm(x_axis)
                    basis=np.column_stack([x_axis,np.cross(ray,x_axis),ray])
                    for roll in c.get('barcode_reader_roll_candidates',[0.]):
                        if canceled():raise InterruptedError('planning canceled')
                        label_from_reader=np.eye(4)
                        label_from_reader[:3,:3]=basis@Rotation.from_rotvec([0.,0.,roll]).as_matrix()@np.diag([1.,-1.,-1.])
                        label_from_reader[:3,3]=c['read_distance']*ray
                        target=pose(barcode_pose)@label_from_reader@np.linalg.inv(pose(c['left_ee_from_reader']))
                        try:
                            trial=self.robot.ik('left',target,goal)
                            self.robot.clearance(trial,c['obstacles'],c['collision_margin'],c['attached_radius'])
                            candidates.append(trial)
                            break
                        except ValueError:continue
                    if candidates:break
                if not candidates:raise ValueError('observed barcode has no reachable reader view candidate')
                # Installation lists candidates in preference order. Stop at the first
                # feasible view so live barcode observations do not age during exhaustive IK.
                goal=candidates[0]
        elif stage == "Scan":
            goal = self.robot.ik("right", c["scan_right_pose"], goal)
            if c.get("object_from_labels"):
                object_from_ee = np.linalg.inv(obj) @ fk[self.robot.ee["right"]]
                predicted_object = pose(c["scan_right_pose"]) @ np.linalg.inv(
                    object_from_ee
                )
                candidates = []
                for label in c["object_from_labels"]:
                    label_from_reader = np.diag([1.0, -1.0, -1.0, 1.0])
                    label_from_reader[2, 3] = c["read_distance"]
                    target = (
                        predicted_object
                        @ pose(label)
                        @ label_from_reader
                        @ np.linalg.inv(pose(c["left_ee_from_reader"]))
                    )
                    try:
                        trial = self.robot.ik("left", target, goal)
                        self.robot.clearance(
                            trial,
                            c["obstacles"],
                            c["collision_margin"],
                            c["attached_radius"],
                        )
                        candidates.append(trial)
                    except ValueError:
                        continue
                if not candidates:
                    raise ValueError("no reachable label presentation")
                goal = min(candidates, key=lambda trial: np.linalg.norm(trial - q))
            else:
                goal = self.robot.ik("left", c["scan_left_pose"], goal)
        elif stage in ('ClearReader', 'PlaceApproach', 'PlaceRetreat') or (stage == 'Place' and placement_enabled(c)):
            if not placement_enabled(c):
                raise ValueError('calibrated head placement is not enabled')
            p = c['placement']
            if stage == 'ClearReader':
                goal[:7] = vector(p['clear_left_joints'], 7)
            elif stage == 'PlaceApproach' and 'hover_right_joints' in p:
                values=vector(p['hover_right_joints'],7)
                if np.any(values < self.robot.limits[14:21,0]) or np.any(values > self.robot.limits[14:21,1]):
                    raise ValueError('placement approach joints outside limits')
                goal[14:21]=values
                actual=self.robot.ee_fk('right',goal);target=pose(p['hover_right_pose'])
                angle=Rotation.from_matrix(target[:3,:3]@actual[:3,:3].T).magnitude()
                if np.linalg.norm(actual[:3,3]-target[:3,3]) > self.robot.ik_position_tolerance_m or angle > self.robot.ik_rotation_tolerance_rad:
                    raise ValueError('placement approach joint target does not match configured pose tolerance')
            else:
                target = pose(p['release_right_pose'] if stage == 'Place' else p['hover_right_pose'])
                goal = self.robot.ik('right', target, goal)
                if stage in ('Place', 'PlaceRetreat'):
                    cartesian_target = target
        elif stage == "Place":
            # Derive held-object relation from measured object and current EE, not initial truth.
            object_from_ee = np.linalg.inv(obj) @ fk[self.robot.ee["right"]]
            goal = self.robot.ik(
                "right", pose(c["destination_pose"]) @ object_from_ee, goal
            )
            goal = self.robot.ik("left", c["retreat_left_pose"], goal)
        elif stage in ("Retreat", "Recover"):
            goal = vector(c["home_joints"], 28).copy()
            # Recovery never opens a possibly loaded hand automatically.
            goal[hand_indices] = q[hand_indices]
        attached = (
            c["attached_radius"]
            if stage in LOADED_STAGES
            else 0.0
        )

        from .placement import trajectory_clearance

        def build(knots):
            scale=c.get('scan_motion_scale',1.) if stage=='ScanPose' else 1.
            if fixed_placement and stage in ('ClearReader','PlaceApproach','Place','PlaceRetreat','Retreat'):
                scale=c['placement'].get('joint_motion_scale',1.)
            acceleration=self.robot.acceleration.copy()*scale**2
            if stage=='Approach':
                acceleration[:7]*=c.get('approach_arm_acceleration_scale',1.)
                acceleration[14:21]*=c.get('approach_arm_acceleration_scale',1.)
            duration_key={'Close':'hand_close_duration_s','Release':'hand_open_duration_s'}.get(stage)
            prepare_route_active=stage in ('Prepare','ReadyPose') and c.get('prepare_conveyor_route') is not None
            if prepare_route_active:
                times=[0.];path=[knots[0]]
                for a,b in zip(knots[:-1],knots[1:]):
                    st,sp=hermite([a,b],self.robot.velocity,acceleration,c['sample_period'])
                    times.extend((times[-1]+st[1:]).tolist());path.extend(sp[1:])
                times=np.asarray(times);path=np.asarray(path)
            elif duration_key and duration_key in c:
                times,path=fixed_hand_path(knots,self.robot.velocity,self.robot.acceleration,
                                           c["sample_period"],c[duration_key])
            elif fixed_placement and c['placement'].get('continuous_motion',False) and stage in ('PlaceApproach','Retreat'):
                times,path=continuous_joint_path(knots,self.robot.velocity*scale,acceleration,c['sample_period'])
            else:
                times, path = hermite(
                    knots, self.robot.velocity*scale, acceleration, c["sample_period"],
                    c.get("lift_minimum_segment_duration",.3) if stage=="Lift" else (c.get("descend_minimum_segment_duration",.3) if stage=="Descend" else .3),
                    corner_blend=c['placement'].get('fixed_joint_corner_blend',0.) if fixed_placement and stage=='PlaceApproach' else 0.
                )
            clearance = 1e6
            for command in path:
                if stage=='AlignBarcode' and c.get('barcode_fixed_wrist'):
                    from .reader_alignment import check_posture
                    check_posture(self.robot,command,c,command=True)
                if canceled():
                    raise InterruptedError("planning canceled")
                clearance = min(
                    clearance,
                    trajectory_clearance(self.robot, c, stage, command, attached),
                )
            if prepare_route_active:
                from .prepare_route import validate_path
                clearance=min(clearance,validate_path(self.robot,path,canceled))
            return times, path, clearance

        knots = [q, goal]
        if fixed_points is not None:
            knots=[q]
            for values in fixed_points:
                waypoint=goal.copy()
                waypoint[14:21]=vector(values,7)
                knots.append(waypoint)
        if stage == "Prepare":
            knots = [q, *[vector(w, 28) for w in c.get("prepare_joint_waypoints", [])], goal]
        if stage in ('Prepare','ReadyPose') and c.get('prepare_conveyor_route') is not None:
            from .prepare_route import knots as prepare_knots
            knots=prepare_knots(c,q,goal,self.robot,canceled)
        if cartesian_target is not None:
            first = fk[self.robot.ee["right"]]
            rotations = Slerp(
                [0, 1], Rotation.from_matrix([first[:3, :3], cartesian_target[:3, :3]])
            )
            count = max(
                2,
                int(
                    np.ceil(
                        np.linalg.norm(first[:3, 3] - cartesian_target[:3, 3]) / (c.get("lift_cartesian_step",.01) if stage=="Lift" else .01)
                    )
                ),
            )
            knots = [q]
            seed = q.copy()
            for fraction in np.linspace(0, 1, count + 1)[1:]:
                if canceled():
                    raise InterruptedError("planning canceled")
                target = np.eye(4)
                target[:3, 3] = (
                    first[:3, 3] * (1 - fraction) + cartesian_target[:3, 3] * fraction
                )
                target[:3, :3] = rotations(fraction).as_matrix()
                seed = self.robot.ik("right", target, seed)
                seed[hand_indices] = (
                    q[hand_indices] * (1 - fraction) + goal[hand_indices] * fraction
                )
                knots.append(seed.copy())
        try:
            times, path, clearance = build(knots)
        except ValueError as error:
            if (
                fixed_points is not None
                or cartesian_target is not None
                or str(error) != "collision clearance"
                or c.get("safe_joint_waypoint") is None
            ):
                raise
            middle = vector(c["safe_joint_waypoint"], 28).copy()
            middle[hand_indices] = q[hand_indices]
            times, path, clearance = build([q, middle, goal])
        if fixed_placement and stage=='PlaceApproach' and c['placement'].get('continuous_motion',False):
            # One arm action: approach/descent, stationary opening, withdrawal/home.
            release=path[-1].copy();opened=release.copy()
            opened[21:28]=np.asarray(c['hand_open'])[7:]
            ht,hq=fixed_hand_path([release,opened],self.robot.velocity,self.robot.acceleration,
                                 c['sample_period'],c['hand_open_duration_s'])
            back=[opened]
            for values in c['placement']['fixed_joint_waypoints']['PlaceRetreat']+c['placement']['fixed_joint_waypoints']['Retreat']:
                waypoint=opened.copy();waypoint[14:21]=values;back.append(waypoint)
            scale=c['placement'].get('joint_motion_scale',1.)
            bt,bq=continuous_joint_path(back,self.robot.velocity*scale,self.robot.acceleration*scale**2,c['sample_period'])
            phases=np.array([times[-1],times[-1]+ht[-1],times[-1]+ht[-1]+bt[-1]])
            times=np.r_[times,phases[0]+ht[1:],phases[1]+bt[1:]]
            path=np.vstack([path,hq[1:],bq[1:]])
            for row in path:
                if canceled():raise InterruptedError('planning canceled')
                clearance=min(clearance,trajectory_clearance(self.robot,c,stage,row,attached))
            return dict(times=times,path=path,minimum_clearance=clearance,
                        ik_calls=self.robot.ik_calls,phase_end_times=phases)
        if stage == 'PlaceApproach' and c.get('placement',{}).get('prevalidate_descent',False) and not c['placement'].get('continuous_motion',False):
            # Reject the approach before dispatch if its subsequent descent cannot be planned.
            try:
                Planner.plan(self,'Place',path[-1],None,canceled=canceled)
            except ValueError as error:
                raise ValueError('placement descent precheck failed before approach dispatch: '+str(error)) from error
        return dict(
            times=times,
            path=path,
            minimum_clearance=clearance,
            ik_calls=self.robot.ik_calls,
        )
