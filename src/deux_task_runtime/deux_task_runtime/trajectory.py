import numpy as np
from .common import seconds


def arrays(trajectory, names):
    if list(trajectory.joint_names) != list(names):
        raise ValueError("trajectory joint order mismatch")
    q = np.asarray([p.positions for p in trajectory.points], dtype=float)
    t = np.asarray([seconds(p.time_from_start) for p in trajectory.points])
    if (
        q.ndim != 2
        or q.shape != (len(t), len(names))
        or len(t) < 2
        or len(t) > 20000
        or not np.isfinite(q).all()
        or not np.isfinite(t).all()
        or t[0] != 0
        or np.any(np.diff(t) <= 0)
    ):
        raise ValueError("malformed trajectory")
    # Only positions are accepted; driver interpolation cannot override checked samples.
    if any(p.velocities or p.accelerations or p.effort for p in trajectory.points):
        raise ValueError("only position samples supported")
    return t, q


def sample(t, q, elapsed):
    return np.array([np.interp(elapsed, t, q[:, i]) for i in range(q.shape[1])])


def validate_limits(trajectory, names, robot):
    from deux_task_core.model import JOINT_NAMES

    t, q = arrays(trajectory, names)
    indices = [JOINT_NAMES.index(n) for n in names]
    limits = robot.limits[indices]
    if np.any(q < limits[:, 0] - robot.joint_limit_tolerance) or np.any(q > limits[:, 1] + robot.joint_limit_tolerance):
        raise ValueError("trajectory joint limit")
    v = np.diff(q, axis=0) / np.diff(t)[:, None]
    a = (
        np.diff(np.vstack([np.zeros(len(names)), v, np.zeros(len(names))]), axis=0)
        / np.r_[np.diff(t), np.diff(t)[-1]][:, None]
    )
    if np.any(abs(v) > robot.velocity[indices] * 1.001):
        raise ValueError("trajectory velocity limit")
    if np.any(abs(a) > robot.acceleration[indices] * 1.01):
        raise ValueError("trajectory acceleration limit")
    return t, q
