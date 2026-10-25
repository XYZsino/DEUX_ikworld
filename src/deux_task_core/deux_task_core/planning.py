"""Observation-based stage goals and time-parametrized monotone Hermite paths."""

import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation, Slerp
from .model import RobotModel, JOINT_NAMES, HAND_NAMES, vector, pose

STAGES = (
    "Approach",
    "Descend",
    "Close",
    "Lift",
    "Scan",
    "AlignBarcode",
    "Place",
    "Release",
    "Retreat",
    "Recover",
)


def validate_scene(config):
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
    vector(config["home_joints"], 28)
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


def hermite(knots, velocity, acceleration, dt):
    knots = np.asarray(knots, dtype=float)
    durations = np.maximum(
        0.3, 2 * np.max(np.abs(np.diff(knots, axis=0)) / velocity, axis=1)
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

    def plan(self, stage, start, object_pose, canceled=lambda: False, barcode_pose=None):
        c = self.config
        if stage not in STAGES:
            raise ValueError("unknown stage")
        q = vector(start, 28).copy()
        obj = pose(object_pose)
        if np.any(obj[:3, 3] < c["workspace"]["min"]) or np.any(
            obj[:3, 3] > c["workspace"]["max"]
        ):
            raise ValueError("object outside workspace")
        fk = self.robot.fk(q)
        goal = q.copy()
        cartesian_target = None
        hand_indices = [JOINT_NAMES.index(n) for n in HAND_NAMES]
        if stage in ('Lift', 'Scan', 'AlignBarcode', 'Place'):
            # Preserve the configured closing setpoint across loaded stages,
            # rather than reducing grip preload to measured stalled positions.
            # Executor still checks this start against measured tolerance.
            q[hand_indices] = c['hand_closed']
            goal[hand_indices] = c['hand_closed']
        if stage in ("Approach", "Descend"):
            target = obj @ pose(c["grasp_object_from_right_ee"])
            if stage == "Approach":
                target[2, 3] += c["approach_height"]
            # Travel to the pre-grasp pose in joint space: a straight Cartesian
            # wrist rotation from home can cross an unreachable configuration.
            # Descend remains Cartesian and all paths retain collision checks.
            if stage == "Descend":
                cartesian_target = target
            goal = self.robot.ik("right", target, goal)
            goal[hand_indices] = c["hand_open"]
        elif stage in ("Close", "Release"):
            goal[hand_indices] = c["hand_closed" if stage == "Close" else "hand_open"]
        elif stage == "Lift":
            target = fk[self.robot.ee["right"]].copy()
            target[2, 3] += c["lift_height"]
            cartesian_target = target
            goal = self.robot.ik("right", target, goal)
        elif stage == "AlignBarcode":
            if barcode_pose is None:
                raise ValueError('observed barcode surface required')
            label_from_reader = np.diag([1., -1., -1., 1.])
            label_from_reader[2, 3] = c['read_distance']
            target = pose(barcode_pose) @ label_from_reader @ np.linalg.inv(pose(c['left_ee_from_reader']))
            goal = self.robot.ik('left', target, goal)
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
            if stage in ("Lift", "Scan", "AlignBarcode", "Place", "Recover")
            else 0.0
        )

        def build(knots):
            times, path = hermite(
                knots, self.robot.velocity, self.robot.acceleration, c["sample_period"]
            )
            clearance = 1e6
            for command in path:
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
        if cartesian_target is not None:
            first = fk[self.robot.ee["right"]]
            rotations = Slerp(
                [0, 1], Rotation.from_matrix([first[:3, :3], cartesian_target[:3, :3]])
            )
            count = max(
                2,
                int(
                    np.ceil(
                        np.linalg.norm(first[:3, 3] - cartesian_target[:3, 3]) / 0.01
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
