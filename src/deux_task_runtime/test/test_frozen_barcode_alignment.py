from types import SimpleNamespace
import pytest
from deux_task_runtime.execution_node import ExecutionNode


def test_no_evidence_allowed_only_during_explicitly_frozen_execution():
    node=SimpleNamespace(config={'barcode_observation_required_during_alignment':False},latest_barcode_target=None)
    plan=SimpleNamespace(barcode_target=SimpleNamespace(request_id='request'))
    ExecutionNode.barcode_guard(node,plan,during_execution=True)
    with pytest.raises(ValueError,match='missing/stale/superseded'):
        ExecutionNode.barcode_guard(node,plan)
    node.config={}
    with pytest.raises(ValueError,match='missing/stale/superseded'):
        ExecutionNode.barcode_guard(node,plan,during_execution=True)
