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


def validate_scene(config):
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
        ('lift_start_lead',.5,.15,.5),
    ):
        value=config.get(key,default)
        if not np.isfinite(value) or not lower<=value<=upper:
            raise ValueError(f'invalid {key}')
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
        if not config["object_from_labels"]:
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
        "attached_radius",
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


def hermite(knots, velocity, acceleration, dt, minimum_segment_duration=0.3):
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


class Planner:
    def __init__(self, config):
        self.config = validate_scene(config)
        self.robot = RobotModel(config["robot"])
        from .ik_backends import attach_backend
        attach_backend(self.robot,config)

    def plan(self, stage, start, object_pose, canceled=lambda: False, barcode_pose=None):
        c = self.config
        if stage not in STAGES:
            raise ValueError("unknown stage")
        q = vector(start, 28).copy()
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
        hand_indices = [JOINT_NAMES.index(n) for n in HAND_NAMES]
        if stage in LOADED_STAGES and stage not in ('Recover','ReadyPose'):
            # Preserve the configured closing setpoint across loaded stages,
            # rather than reducing grip preload to measured stalled positions.
            # Executor still checks this start against measured tolerance.
            q[hand_indices] = c['hand_closed']
            goal[hand_indices] = c['hand_closed']
        if stage == "Prepare":
            goal = vector(c["home_joints"], 28).copy()
        elif stage == 'ReadyPose':
            # Keep hand state while returning the arms. Preserve closing preload
            # if the measured fingers are already near the closed command.
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
        elif stage in ("Approach", "Descend"):
            target = obj @ pose(c["grasp_object_from_right_ee"])
            if stage == "Approach":
                target[2, 3] += c["approach_height"]
            # Travel to the pre-grasp pose in joint space: a straight Cartesian
            # wrist rotation from home can cross an unreachable configuration.
            # Descend remains Cartesian and all paths retain collision checks.
            if stage == "Descend":
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
            if c.get('barcode_fixed_wrist'):
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

        def build(knots):
            scale=c.get('scan_motion_scale',1.) if stage=='ScanPose' else 1.
            times, path = hermite(
                knots, self.robot.velocity*scale, self.robot.acceleration*scale**2, c["sample_period"],
                c.get("lift_minimum_segment_duration",.3) if stage=="Lift" else .3
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
                    self.robot.clearance(
                        command, c["obstacles"], c["collision_margin"], attached
                    ),
                )
            return times, path, clearance

        knots = [q, goal]
        if stage == "Prepare":
            knots = [q, *[vector(w, 28) for w in c.get("prepare_joint_waypoints", [])], goal]
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
                cartesian_target is not None
                or str(error) != "collision clearance"
                or c.get("safe_joint_waypoint") is None
            ):
                raise
            middle = vector(c["safe_joint_waypoint"], 28).copy()
            middle[hand_indices] = q[hand_indices]
            times, path, clearance = build([q, middle, goal])
        return dict(
            times=times,
            path=path,
            minimum_clearance=clearance,
            ik_calls=self.robot.ik_calls,
        )
