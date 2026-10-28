"""Dispatch remains available with temporal search or all candidates disabled."""
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')

@pytest.mark.parametrize('candidates,timed,expected',[(True,True,'time'),(True,False,'grip'),(False,False,'basic'),(False,True,'basic')])
def test_approach_search_toggle(monkeypatch,candidates,timed,expected):
    import deux_task_runtime.motion_planner_node as runtime
    import deux_task_core.time_candidates as temporal
    import deux_task_core.grasp_candidates as grips
    import deux_task_core.conveyor as conveyor
    config={'conveyor_tracking':{'candidate_search':{'enabled':candidates,'time_search':{'enabled':timed}}}}
    monkeypatch.setattr(runtime,'cached_planner',lambda *a:SimpleNamespace(config=config))
    monkeypatch.setattr(temporal,'search',lambda *a:'time')
    monkeypatch.setattr(grips,'select_candidate',lambda *a:'grip')
    monkeypatch.setattr(conveyor,'plan_conveyor',lambda *a:'basic')
    assert runtime.compute_plan(config,'version','Approach',None,None,motion=([0,-.08,0,0],.5))==expected
