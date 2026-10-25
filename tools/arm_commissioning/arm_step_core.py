"""Small joint-space commissioning plans; no ROS or hardware side effects."""

import hashlib
import json
import math
import xml.etree.ElementTree as ET
import numpy as np

ARM_NAMES = tuple(f"arm_{s}_joint{i}" for s in ("l", "r") for i in range(1, 8))
TERMINAL = {4, 5, 6}  # action_msgs/GoalStatus: succeeded, canceled, aborted


def joint_vector(msg):
    if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
        raise ValueError("malformed joint feedback")
    values = dict(zip(msg.name, msg.position))
    out = np.array([values[n] for n in ARM_NAMES], dtype=float)
    if not np.isfinite(out).all():
        raise ValueError("nonfinite joint feedback")
    return out


def bounds(urdf):
    root = ET.fromstring(urdf)
    by_name = {j.get("name"): j for j in root.findall("joint")}
    result = []
    for name in ARM_NAMES:
        j = by_name[name]
        limit = j.find("limit")
        if j.get("type") != "revolute" or limit is None:
            raise ValueError("explicit revolute limits required: " + name)
        lo, hi = float(limit.get("lower")), float(limit.get("upper"))
        if not math.isfinite(lo + hi) or lo >= hi:
            raise ValueError("invalid limits: " + name)
        result.append((lo, hi))
    return np.array(result)


def make_plan(start, joint, delta_deg, duration, urdf):
    start = np.asarray(start, dtype=float)
    if start.shape != (14,) or not np.isfinite(start).all():
        raise ValueError("14 measured arm positions required")
    if (
        joint not in ARM_NAMES
        or not math.isfinite(delta_deg)
        or not 0 < abs(delta_deg) <= 2
    ):
        raise ValueError("one named arm joint, nonzero delta <=2 degrees required")
    if not math.isfinite(duration) or not 3 <= duration <= 10:
        raise ValueError("duration must be 3..10 seconds")
    limits = bounds(urdf)
    target = start.copy()
    target[ARM_NAMES.index(joint)] += math.radians(delta_deg)
    if np.any(np.minimum(start, target) < limits[:, 0]) or np.any(
        np.maximum(start, target) > limits[:, 1]
    ):
        raise ValueError("start/target outside URDF limits")
    times = np.linspace(0, duration, math.ceil(duration / 0.02) + 1)
    u = times / duration
    blend = 10 * u**3 - 15 * u**4 + 6 * u**5
    path = start[None, :] + blend[:, None] * (target - start)[None, :]
    return dict(
        joint_names=list(ARM_NAMES),
        joint=joint,
        delta_deg=delta_deg,
        duration_s=duration,
        start_rad=start.tolist(),
        target_rad=target.tolist(),
        times_s=times.tolist(),
        positions_rad=path.tolist(),
        peak_velocity_rad_s=float(1.875 * abs(math.radians(delta_deg)) / duration),
        urdf_sha256=hashlib.sha256(urdf.encode()).hexdigest(),
    )


def fingerprint(plan):
    return hashlib.sha256(
        json.dumps(plan, sort_keys=True, allow_nan=False).encode()
    ).hexdigest()


def cancellation_complete(handle, wait, result_future=None, outcome=None):
    """Reuse the result future and distinguish cancellation from an existing terminal result.

    This prevents duplicate cancellation of a known terminal goal. It cannot
    eliminate races between the driver aborting and a cancel request in flight.
    """
    if outcome is None:
        outcome = {}
    outcome["cancel_requested"] = False
    if handle is None or not handle.accepted:
        return None
    future = result_future if result_future is not None else handle.get_result_async()
    if future.done():
        result = future.result()
        if result.status in TERMINAL:
            return result
    if getattr(handle, "status", None) not in TERMINAL:
        outcome["cancel_requested"] = True
        response = wait(handle.cancel_goal_async(), 2.0)
        outcome["cancel_response_code"] = int(response.return_code)
        outcome["cancel_acknowledged"] = bool(response.goals_canceling)
    result = wait(future, 3.0)
    if result.status not in TERMINAL:
        raise RuntimeError("driver did not reach terminal status after cancel")
    return result


def status_problems(snapshot, monotonic_now, ros_now):
    problems = []
    for key in ("joints", "controller", "stiffness"):
        stream = snapshot.get(key)
        if stream is None:
            problems.append(key + " missing")
        elif (
            not 0 <= monotonic_now - stream["received"] <= 0.2
            or not 0 <= ros_now - stream["stamp"] <= 0.2
        ):
            problems.append(key + " stale/future")
    if snapshot.get("stiffness_applied") != "setpoint":
        problems.append("stiffness state not in ordinary setpoint operation")
    publishers = snapshot.get("command_publishers", [])
    if publishers:
        problems.append("competing trajectory publishers: " + ", ".join(publishers))
    if snapshot.get("active_goals"):
        problems.append("controller has active Action goal(s)")
    if not snapshot.get("controller_active", False):
        problems.append("trajectory controller not active/verified")
    if snapshot.get("estop_seen", False):
        problems.append("estop latched during this process; no automatic reset")
    return problems


# These are commissioning-tool limits, not robot manufacturer specifications.
TOLERANCES = {
    "start_reference_error_deg": 0.5,
    "monitor_tracking_error_deg": 0.4,
    "driver_path_error_deg": 0.5,
    "goal_position_error_deg": 0.25,
    "goal_time_allowance_s": 2.0,
    "preview_max_age_s": 60.0,
    "preview_start_drift_rad": 0.002,
}


def motion_guard(actual, expected):
    actual = np.asarray(actual, dtype=float)
    expected = np.asarray(expected, dtype=float)
    if (
        actual.shape != (14,)
        or expected.shape != (14,)
        or not np.isfinite([actual, expected]).all()
    ):
        raise RuntimeError("invalid measured/expected posture")
    if np.max(np.abs(actual - expected)) > np.deg2rad(
        TOLERANCES["monitor_tracking_error_deg"]
    ):
        raise RuntimeError("following error >0.4deg; stop before driver 0.5deg abort")
