from types import SimpleNamespace
import numpy as np
import pytest
from arm_step_core import (
    make_plan,
    ARM_NAMES,
    status_problems,
    cancellation_complete,
    motion_guard,
)


@pytest.fixture
def urdf():
    return (
        "<robot>"
        + "".join(
            f'<joint name="{n}" type="revolute"><limit lower="-1" upper="1"/></joint>'
            for n in ARM_NAMES
        )
        + "</robot>"
    )


def test_bounded_single_joint_motion(urdf):
    p = make_plan(np.zeros(14), "arm_r_joint7", 1, 3, urdf)
    q = np.array(p["positions_rad"])
    t = np.array(p["times_s"])
    assert np.all(q[:, :13] == 0)
    assert q[0, 13] == 0 and q[-1, 13] == pytest.approx(np.deg2rad(1))
    assert np.max(np.diff(t)) <= 0.02000001
    assert np.all(np.diff(q[:, 13]) >= 0)
    assert np.max(np.diff(q[:, 13]) / np.diff(t)) <= p["peak_velocity_rad_s"]


@pytest.mark.parametrize(
    "delta,duration",
    [(3, 3), (5, 3), (10, 10), (float("nan"), 3), (1, 0), (1, 11), (0, 3)],
)
def test_refuses_experimental_or_invalid_moves(urdf, delta, duration):
    with pytest.raises(ValueError):
        make_plan(np.zeros(14), "arm_r_joint7", delta, duration, urdf)


def test_joint_limit(urdf):
    q = np.zeros(14)
    q[-1] = 0.999
    with pytest.raises(ValueError, match="limits"):
        make_plan(q, "arm_r_joint7", 1, 3, urdf)


def test_tracking_guard_covers_noncommanded_joints():
    actual = np.zeros(14)
    actual[0] = np.deg2rad(0.5)
    with pytest.raises(RuntimeError, match="following"):
        motion_guard(actual, np.zeros(14))
    actual[0] = float("nan")
    with pytest.raises(RuntimeError, match="invalid"):
        motion_guard(actual, np.zeros(14))


def test_gate_rejects_conflict_stale_and_estop():
    fresh = {"received": 10.0, "stamp": 100.0}
    s = {
        "joints": fresh,
        "controller": fresh,
        "stiffness": fresh,
        "stiffness_applied": "setpoint",
        "controller_active": True,
    }
    assert status_problems(s, 10.1, 100.1) == []
    assert "stale" in " ".join(status_problems(s, 10.3, 100.3))
    s.update(command_publishers=["/leader"], estop_seen=True, active_goals=["other"])
    failures = " ".join(status_problems(s, 10.1, 100.1))
    assert all(w in failures for w in ["competing", "estop", "active"])


def test_cancel_ack_requires_terminal_result_and_reuses_future():
    calls = []
    future = SimpleNamespace(done=lambda: False)

    def unexpected():
        raise AssertionError("must reuse original result future")

    h = SimpleNamespace(
        accepted=True,
        status=2,
        cancel_goal_async=lambda: "ack",
        get_result_async=unexpected,
    )

    def wait(f, t):
        calls.append(f)
        return (
            SimpleNamespace(status=5)
            if f is future
            else SimpleNamespace(return_code=0, goals_canceling=["id"])
        )

    outcome = {}
    assert cancellation_complete(h, wait, future, outcome).status == 5
    assert (
        calls == ["ack", future]
        and outcome["cancel_requested"]
        and outcome["cancel_acknowledged"]
    )


def test_known_terminal_result_not_canceled():
    result = SimpleNamespace(status=6)
    future = SimpleNamespace(done=lambda: True, result=lambda: result)

    def unexpected(*args):
        raise AssertionError("terminal goal must not be canceled or requested again")

    h = SimpleNamespace(
        accepted=True,
        status=6,
        cancel_goal_async=unexpected,
        get_result_async=unexpected,
    )
    outcome = {}
    assert cancellation_complete(h, unexpected, future, outcome) is result
    assert outcome["cancel_requested"] is False


def test_terminal_timeout_propagates():
    future = SimpleNamespace(done=lambda: False)
    h = SimpleNamespace(accepted=True, status=2, cancel_goal_async=lambda: "ack")

    def wait(f, t):
        if f is future:
            raise TimeoutError("not stopped")
        return SimpleNamespace(return_code=0, goals_canceling=["id"])

    with pytest.raises(TimeoutError):
        cancellation_complete(h, wait, future)
