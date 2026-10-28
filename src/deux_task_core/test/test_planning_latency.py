import pytest
from deux_task_core.planning_latency import PlanningLatency, settings


def test_real_budget_includes_both_computations_and_dispatch():
    budget=PlanningLatency({})
    assert budget.reserve_start(10.)==pytest.approx(10.+(3.+.5)*1.25+.25)
    assert not budget.can_validate(11.,10.5)
    assert budget.can_dispatch(11.,10.7)
    assert not budget.can_dispatch(11.,10.8)


def test_simulation_clock_converts_wall_compute_without_compressing_motion():
    budget=PlanningLatency({},use_sim_time=True)
    for i in range(20):budget.sample(10.+i*.05,100.+i*.1)
    assert budget.clock_rate==pytest.approx(.5)
    start=budget.reserve_start(11.)
    arrival=start+2.
    budget.observe('search',3.5)
    assert budget.can_validate(start,11.+1.75)
    # Dispatch keeps the original start: no 2s -> 0.7s shortening.
    assert arrival-start==2.
    assert budget.can_dispatch(start,11.+1.75+.1)


def test_overrun_requires_new_future_start_with_measured_cost():
    budget=PlanningLatency({})
    first=budget.reserve_start(10.)
    budget.observe('search',5.)
    assert not budget.can_validate(first,15.)
    second=budget.reserve_start(15.)
    assert second>first and second>15.+5.+.5
    assert budget.wall_budget['search']==5.


def test_measured_budget_replaces_initial_prior_and_retains_recent_slow_sample():
    config={'conveyor_tracking':{'candidate_search':{'time_search':{'planning_budget':{'measurement_window':2}}}}}
    budget=PlanningLatency(config)
    budget.observe('search',1.2)
    assert budget.wall_budget['search']==1.2
    budget.observe('search',2.)
    budget.observe('search',.8)
    assert budget.wall_budget['search']==2.
    budget.observe('search',.9)
    assert budget.wall_budget['search']==.9


def test_real_clock_never_uses_simulation_rate_and_reset_discards_history():
    budget=PlanningLatency({})
    budget.sample(1.,10.);budget.sample(2.,12.)
    assert budget.clock_rate==1.
    budget.use_sim_time=True
    assert budget.clock_rate==.5
    budget.sample(0.,13.)
    assert budget.clock_rate==1. and len(budget.samples)==1


@pytest.mark.parametrize('key,value', [('dispatch_lead_s',.01),('safety_factor',.5),
    ('max_research_attempts',True),('initial_search_wall_s',float('nan'))])
def test_invalid_budgets_rejected(key,value):
    with pytest.raises(ValueError):
        settings({'conveyor_tracking':{'candidate_search':{'time_search':{'planning_budget':{key:value}}}}})
