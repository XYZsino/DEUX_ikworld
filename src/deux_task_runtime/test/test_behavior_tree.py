import asyncio
import pytest
from deux_task_runtime.behavior_tree import Leaf, Sequence, Retry, Failure, Status


def test_memory_sequence_bounded_retry_and_failure_propagation():
    seen = []
    async def start(): seen.append('start')
    async def scan():
        seen.append('scan')
        if seen.count('scan') < 3: raise Failure('no read')
    async def finish(): seen.append('finish')
    tree = Sequence('root',[Leaf('start',start),Retry('retry',Leaf('scan',scan),3),Leaf('finish',finish)])
    asyncio.run(tree.run())
    assert seen == ['start','scan','scan','scan','finish']
    assert tree.status == Status.SUCCESS
    async def canceled(): raise InterruptedError('cancel')
    tree = Retry('retry',Leaf('cancel',canceled),3)
    with pytest.raises(InterruptedError): asyncio.run(tree.run())
    assert tree.status == Status.FAILURE
