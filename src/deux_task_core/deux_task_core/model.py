"""URDF kinematics and explicit conservative collision envelopes, in base frame."""

from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ARM_NAMES = tuple(f"arm_{s}_joint{i}" for s in ("l", "r") for i in range(1, 8))
PARTS = (
    "thumb_joint_1",
    "thumb_joint_2",
    "thumb_joint_3",
    "index_joint_1",
    "index_joint_2",
    "third_joint_1",
    "third_joint_2",
)
HAND_NAMES = tuple(f"{side}_hand_{p}" for side in ("left", "right") for p in PARTS)
JOINT_NAMES = tuple(
    n
    for side in ("left", "right")
    for n in (
        *[f"arm_{side[0]}_joint{i}" for i in range(1, 8)],
        *[f"{side}_hand_{p}" for p in PARTS],
    )
)


def vector(value, size):
    a = np.asarray(value, dtype=float)
    if a.shape != (size,) or not np.isfinite(a).all():
        raise ValueError(f"expected finite vector of length {size}")
    return a


def pose(value):
    a = np.asarray(value, dtype=float)
    if (
        a.shape != (4, 4)
        or not np.isfinite(a).all()
        or not np.allclose(a[3], [0, 0, 0, 1])
        or not np.allclose(a[:3, :3].T @ a[:3, :3], np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(a[:3, :3]), 1)
    ):
        raise ValueError("invalid rigid pose")
    return a


def origin(element):
    t = np.eye(4)
    if element is not None:
        t[:3, 3] = vector(element.get("xyz", "0 0 0").split(), 3)
        t[:3, :3] = Rotation.from_euler(
            "xyz", vector(element.get("rpy", "0 0 0").split(), 3)
        ).as_matrix()
    return t


class RobotModel:
    def __init__(self, config):
        self.config = config
        root = ET.fromstring(Path(config["urdf_path"]).read_text())
        self.base = config["base_frame"]
        self.ee = config["ee_links"]
        self.joints = {}
        self.children = {}
        self.fixed = config.get("fixed_joints", {})
        for j in root.findall("joint"):
            name, kind = j.attrib["name"], j.attrib["type"]
            if j.find("mimic") is not None:
                raise ValueError("expand URDF mimic joints explicitly before planning")
            parent, child = (
                j.find("parent").attrib["link"],
                j.find("child").attrib["link"],
            )
            axis = j.find("axis")
            axis = (
                vector(axis.get("xyz", "1 0 0").split(), 3)
                if axis is not None
                else np.array([1.0, 0, 0])
            )
            if np.linalg.norm(axis) == 0:
                raise ValueError("zero joint axis")
            limit = j.find("limit")
            self.joints[name] = dict(
                kind=kind,
                parent=parent,
                child=child,
                origin=origin(j.find("origin")),
                axis=axis / np.linalg.norm(axis),
                limit=limit,
            )
            self.children.setdefault(parent, []).append(name)
        limits = []
        velocity = []
        for name in JOINT_NAMES:
            j = self.joints[name]
            lim = j["limit"]
            if j["kind"] != "revolute" or lim is None:
                raise ValueError(f"explicit revolute joint limits required: {name}")
            limits.append([float(lim.attrib["lower"]), float(lim.attrib["upper"])])
            velocity.append(float(lim.attrib["velocity"]))
        self.limits = np.array(limits)
        self.velocity = np.array(velocity)
        if (
            not np.isfinite(self.limits).all()
            or np.any(self.limits[:, 0] >= self.limits[:, 1])
            or np.any(self.velocity <= 0)
        ):
            raise ValueError("invalid URDF limits")
        self.acceleration = vector(config["acceleration_limits"], 28)
        if np.any(self.acceleration <= 0):
            raise ValueError("positive acceleration limits required")
        self.joint_limit_tolerance = float(config.get('joint_limit_tolerance', 1e-7))
        if not np.isfinite(self.joint_limit_tolerance) or not 0 <= self.joint_limit_tolerance <= .001:
            raise ValueError('joint limit numerical tolerance must be within 0..0.001 rad')
        self.spheres = config["collision_spheres"]
        if not self.spheres:
            raise ValueError("calibrated collision envelopes required")
        for s in self.spheres:
            vector(s["center"], 3)
            if not np.isfinite(s["radius"]) or s["radius"] <= 0:
                raise ValueError("invalid collision radius")
        self.ignore = {frozenset(p) for p in config.get("allowed_collision_pairs", [])}
        self.ik_calls = 0
        links = self.fk(np.mean(self.limits, axis=1))
        if any(
            link not in links
            for link in [*self.ee.values(), *[s["link"] for s in self.spheres]]
        ):
            raise ValueError("EE/collision link disconnected from base")

    def fk(self, q):
        values = dict(zip(JOINT_NAMES, vector(q, 28)))
        transforms = {self.base: np.eye(4)}

        def visit(parent):
            for name in self.children.get(parent, []):
                j = self.joints[name]
                motion = np.eye(4)
                if j["kind"] != "fixed":
                    if name not in values and name not in self.fixed:
                        raise ValueError(f"unmeasured joint: {name}")
                    value = float(values.get(name, self.fixed.get(name)))
                    if not np.isfinite(value):
                        raise ValueError("invalid fixed joint")
                    if j["kind"] in ("revolute", "continuous"):
                        motion[:3, :3] = Rotation.from_rotvec(
                            j["axis"] * value
                        ).as_matrix()
                    elif j["kind"] == "prismatic":
                        motion[:3, 3] = j["axis"] * value
                    else:
                        raise ValueError("unsupported joint type")
                transforms[j["child"]] = transforms[parent] @ j["origin"] @ motion
                visit(j["child"])

        visit(self.base)
        return transforms

    def ik(self, side, target, seed):
        target = pose(target)
        q = vector(seed, 28).copy()
        indices = np.array(
            [JOINT_NAMES.index(n) for n in ARM_NAMES[: 7 if side == "left" else 14]]
        )
        indices = indices[:7] if side == "left" else indices[7:]
        self.ik_calls += 1

        def residual(arm):
            trial = q.copy()
            trial[indices] = arm
            actual = self.fk(trial)[self.ee[side]]
            return np.r_[
                actual[:3, 3] - target[:3, 3],
                0.15
                * Rotation.from_matrix(target[:3, :3] @ actual[:3, :3].T).as_rotvec(),
            ]

        result = least_squares(
            residual,
            np.clip(
                q[indices],
                self.limits[indices, 0] + 1e-8,
                self.limits[indices, 1] - 1e-8,
            ),
            bounds=(self.limits[indices, 0], self.limits[indices, 1]),
            max_nfev=180,
        )
        q[indices] = result.x
        r = residual(result.x)
        if np.linalg.norm(r[:3]) > 0.002 or np.linalg.norm(r[3:]) / 0.15 > 0.02:
            raise ValueError("IK unreachable")
        return q

    def clearance(self, q, obstacles, margin, attached_radius=0.0):
        q = vector(q, 28)
        if np.any(q < self.limits[:, 0] - self.joint_limit_tolerance) or np.any(q > self.limits[:, 1] + self.joint_limit_tolerance):
            raise ValueError("joint limit")
        links = self.fk(q)
        spheres = [
            (
                s["link"],
                (links[s["link"]] @ np.r_[s["center"], 1])[:3],
                float(s["radius"]),
            )
            for s in self.spheres
        ]
        if attached_radius:
            spheres.append(
                ("attached_object", links[self.ee["right"]][:3, 3], attached_radius)
            )
        minimum = float("inf")
        for i, (name, c, r) in enumerate(spheres):
            for other, d, s in spheres[:i]:
                if name == other or frozenset((name, other)) in self.ignore:
                    continue
                # Attached envelope encloses the grasping hand by explicit configuration.
                if name == "attached_object" and other in self.config.get(
                    "attachment_touch_links", []
                ):
                    continue
                minimum = min(minimum, float(np.linalg.norm(c - d) - r - s))
            for box in obstacles:
                lo, hi = vector(box["min"], 3), vector(box["max"], 3)
                distance = np.linalg.norm(c - np.clip(c, lo, hi)) - r
                minimum = min(minimum, float(distance))
        if minimum < margin:
            raise ValueError("collision clearance")
        return minimum if np.isfinite(minimum) else 1e6
