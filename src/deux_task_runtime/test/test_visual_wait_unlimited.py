import asyncio
from types import SimpleNamespace
import pytest
import deux_task_runtime.head_task_node as module

def test_visual_wait_outlives_previous_deadline(monkeypatch):
    state={'ticks':0}
    async def tick(node):state['ticks']+=1
    monkeypatch.setattr(module,'pause',tick)
    node=SimpleNamespace(guard=lambda *a:None,now=lambda:state['ticks']*100.,wall_timeout=lambda t:t)
    asyncio.run(module.HeadTaskNode.await_condition(node,lambda:state['ticks']>=4,None,None,.001))
    assert state['ticks']==4

def test_visual_wait_still_checks_cancel_guard(monkeypatch):
    def guard(*a):raise InterruptedError('canceled')
    node=SimpleNamespace(guard=guard)
    with pytest.raises(InterruptedError,match='canceled'):
        asyncio.run(module.HeadTaskNode.await_condition(node,lambda:False,None,None))
