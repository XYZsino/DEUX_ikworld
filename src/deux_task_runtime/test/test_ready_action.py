"""Admission serialization applies across RunTask and MoveToReady."""
import asyncio
from threading import Lock
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from rclpy.action import GoalResponse
from deux_task_msgs.action import MoveToReady
from deux_task_runtime.common import ExclusiveAction


@pytest.mark.parametrize('outcome',['success','failure','cancel'])
def test_shared_gate_rejects_other_action_and_releases_after_terminal(outcome):
    gate=Lock()
    def action():
        a=ExclusiveAction.__new__(ExclusiveAction)
        a.gate=gate;a.busy=False;a.action=MoveToReady;a.validate=lambda _:None
        a.node=SimpleNamespace(get_logger=lambda:SimpleNamespace(warning=lambda _:None))
        return a
    run,ready=action(),action()
    assert run.accept(None)==GoalResponse.ACCEPT
    assert ready.accept(None)==GoalResponse.REJECT
    terminal=[]
    async def execute(goal):
        assert ready.accept(None)==GoalResponse.REJECT
        if outcome=='failure':raise ValueError('planned failure')
        return MoveToReady.Result(success=True)
    run.execute=execute
    goal=SimpleNamespace(is_cancel_requested=outcome=='cancel',
        succeed=lambda:terminal.append('success'),abort=lambda:terminal.append('failure'),
        canceled=lambda:terminal.append('cancel'))
    result=asyncio.run(run.run(goal))
    assert terminal==[outcome] and not run.busy
    assert result.success==(outcome=='success')
    assert ready.accept(None)==GoalResponse.ACCEPT
    gate.release()


def test_rejected_validation_releases_shared_admission_gate():
    a=ExclusiveAction.__new__(ExclusiveAction);a.gate=Lock();a.busy=False
    a.node=SimpleNamespace(get_logger=lambda:SimpleNamespace(warning=lambda _:None))
    def reject(_):raise ValueError('stale measured state')
    a.validate=reject
    assert a.accept(None)==GoalResponse.REJECT
    assert a.gate.acquire(blocking=False)
    a.gate.release()
