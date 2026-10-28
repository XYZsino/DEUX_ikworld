import json
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from deux_task_gui.node import TaskGui


def test_grid_survives_selected_revalidation_but_rejects_unrelated_update():
    node=SimpleNamespace(time_candidates=None)
    grid=dict(time_search=True,time_bins_s=[1.,1.25],target_id='a',scope='search',candidates=[])
    TaskGui.on_candidates(node,SimpleNamespace(data=json.dumps(grid)))
    TaskGui.on_candidates(node,SimpleNamespace(data=json.dumps(dict(time_search=True,target_id='b',scope='fresh',candidates=[dict(feasible=False)]))))
    assert 'latest_validation' not in node.time_candidates
    TaskGui.on_candidates(node,SimpleNamespace(data=json.dumps(dict(time_search=True,target_id='a',scope='fresh',candidates=[dict(feasible=False)]))))
    assert node.time_candidates['time_bins_s']==[1.,1.25]
    assert node.time_candidates['validation_ok'] is False
    TaskGui.on_candidates(node,SimpleNamespace(data='not-json'))
    assert node.time_candidates['validation_ok'] is False
