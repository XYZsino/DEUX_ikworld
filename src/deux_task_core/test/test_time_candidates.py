import numpy as np
import pytest
from test_grasp_candidates import fixture
from deux_task_core.time_candidates import search,revalidate,validate_time_search,predicted_std
from deux_task_core.conveyor import plan_conveyor,plan_track_grasp


def setup():
    p,q,obj=fixture()
    p.config['conveyor_tracking'].update(prediction_acceleration_std=.001)
    p.config['conveyor_tracking']['candidate_search']['time_search']=dict(enabled=True,
        arrival_min_s=1.,arrival_max_s=2.,step_s=.25,handoff_s=.5,full_path_candidates=3,max_prediction_std_m=.04)
    return p,q,obj


def test_grid_uses_one_snapshot_and_shortlist_is_bounded():
    p,q,obj=setup();q0=q.copy();obj0=obj.copy()
    data=search(p,q,obj,[0,-.08,0,0],.5,np.eye(8)*1e-6)
    report=data['candidate_report']
    assert report['screened_count']==10
    assert report['full_path_count']<=3
    assert report['batch_mode']=='serial_compatibility'
    np.testing.assert_array_equal(q,q0);np.testing.assert_array_equal(obj,obj0)
    assert 'planning_error' not in data
    timing=data['timing']
    assert timing['grasp_start_lead_s']-timing['arrival_lead_s']==pytest.approx(.5)
    assert data['times'][-1]+.5==pytest.approx(timing['arrival_lead_s'])
    # Same absolute object prediction after moving the acquisition clock forward.
    moved=obj.copy();moved[1,3]-=.08*.1
    new=dict(timing,arrival_lead_s=timing['arrival_lead_s']-.1,grasp_start_lead_s=timing['grasp_start_lead_s']-.1)
    refreshed=revalidate(p,q,moved,[0,-.08,0,0],.4,data['candidate_id'],new)
    assert 'planning_error' not in refreshed
    np.testing.assert_allclose(refreshed['path'],data['path'],atol=1e-6)


def test_uncertainty_rejects_before_full_paths():
    p,q,obj=setup()
    data=search(p,q,obj,[0,-.08,0,0],.5,np.eye(8)*.1)
    assert data['candidate_report']['full_path_count']==0
    assert all(r['rejected_stage']=='Prediction' for r in data['candidate_report']['candidates'])
    assert 'planning_error' in data
    with pytest.raises(ValueError,match='covariance'):predicted_std(-np.eye(8),1.,p.config)


def test_fixed_arrival_does_not_silently_retime():
    p,q,obj=setup();p.config['conveyor_tracking']['fixed_approach_duration_s']=.01
    with pytest.raises(ValueError,match='arrival time'):plan_conveyor(p,'Approach',q,obj,[0,-.08,0,0],.5)
    p.config['conveyor_tracking']['fixed_grasp_durations_s']=[0.,.01,.01]
    with pytest.raises(ValueError,match='closing deadline'):plan_track_grasp(p,q,obj,[0,-.08,0,0],.5)


def test_time_grid_bounds():
    p,_,_=setup();validate_time_search(p.config)
    p.config['conveyor_tracking']['candidate_search']['time_search']['step_s']=.001
    with pytest.raises(ValueError,match='time grid'):validate_time_search(p.config)


def test_passed_arrival_is_rejected_without_moving_deadline():
    p,q,obj=setup()
    with pytest.raises(ValueError,match='deadline elapsed'):
        revalidate(p,q,obj,[0,-.08,0,0],2.,'nominal',dict(arrival_lead_s=1.,grasp_start_lead_s=2.5,phase_durations=[0.,.8,1.]))


def test_uncertainty_uses_same_independent_belt_prior_as_forecast():
    p,_,_=setup();cfg=p.config['conveyor_tracking']
    cov=np.eye(8)*1e-6;cov[4:,4:]=np.eye(4)*.1
    image_std=predicted_std(cov,5.,p.config)
    cfg.update(occluded_grasp_enabled=True,belt_velocity_std_m_s=.002)
    belt_std=predicted_std(cov,5.,p.config)
    assert image_std>1. and belt_std<.02
    cov[0,0]=.1
    assert predicted_std(cov,5.,p.config)>.3  # Measured pose uncertainty is retained.


def test_overlap_shortens_total_and_screening_uses_executable_finger_duration():
    from deux_task_core.grasp_timing import initial_durations,knots
    from deux_task_core.model import HAND_NAMES,JOINT_NAMES
    p,q,obj=setup()
    p.config['hand_pregrasp']=list(p.config['hand_open'])
    p.config['hand_closed']=list(p.config['hand_open'])
    p.config['hand_closed'][10]+=.7
    hands=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    p.robot.acceleration[hands]=3.
    sequential=initial_durations(p.config,p.robot)
    p.config['conveyor_tracking']['close_descent_overlap']=.5
    overlap=initial_durations(p.config,p.robot)
    assert sum(overlap)<sum(sequential)-.2
    _,begin=knots(overlap,.5)
    assert begin<sum(overlap[:2])
    data=search(p,q,obj,[0,-.08,0,0],.5)
    row=data['candidate_report']['candidates'][0]
    assert row['representative_close_lead_s']-row['grasp_start_lead_s']==pytest.approx(sum(overlap))


def test_short_handoff_requires_prefetch():
    p,_,_=setup();cfg=p.config['conveyor_tracking']['candidate_search']['time_search']
    p.config['conveyor_tracking']['start_lead']=.8;cfg['handoff_s']=.35
    with pytest.raises(ValueError,match='handoff'):validate_time_search(p.config)
    cfg['prefetch_grasp']=True;validate_time_search(p.config)
