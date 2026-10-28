from copy import deepcopy
import numpy as np
import pytest
from test_conveyor import setup_plan
from deux_task_core.grasp_candidates import (candidate_config, metrics, select_candidate,
                                           revalidate_grasp, validate)
from deux_task_core.planning import Planner


def fixture():
    planner, q, obj = setup_plan()
    planner.config.update(lift_height=.10, attached_radius=.05, perception_policy='head_rgbd')
    tracking = planner.config['conveyor_tracking']
    tracking.update(workspace_policy='ik', start_lead=.4, intercept_wait_s=.4,
                    intercept_trigger_distance_m=.025)
    tracking['candidate_search'] = dict(enabled=True, selection_start_lead_s=1.,
        joint_travel_weight=1., clearance_weight=2., clearance_scale_m=.4,
        candidates=[dict(id='nominal', yaw_deg=0., approach_offset_base_m=[0.,0.,0.]),
                    dict(id='tilted', yaw_deg=8., approach_offset_base_m=[.015,0.,0.])])
    return planner, q, obj


def test_lift_failure_rejects_candidate_before_any_approach_is_selected(monkeypatch):
    planner, q, obj = fixture()
    original = Planner.plan
    calls = []
    def lift(self, stage, start, target):
        calls.append(stage)
        if self.config['conveyor_tracking']['approach_offset_base_m'][0] == 0:
            raise ValueError('Lift collision clearance')
        return original(self, stage, start, target)
    monkeypatch.setattr(Planner, 'plan', lift)
    before = deepcopy(planner.config)
    data = select_candidate(planner, q, obj, [0.,-.08,0.,0.], .5)
    assert data['candidate_id'] == 'tilted'
    reports = data['candidate_report']['candidates']
    assert reports[0]['rejected_stage'] == 'Lift'
    assert reports[1]['feasible']
    assert calls == ['Lift', 'Lift']
    np.testing.assert_allclose(planner.config['grasp_object_from_right_ee'], before['grasp_object_from_right_ee'])


def test_all_failed_candidates_return_no_trajectory(monkeypatch):
    planner, q, obj = fixture()
    def impossible(*args, **kwargs):
        raise ValueError('IK unreachable')
    monkeypatch.setattr(planner.robot, 'ik', impossible)
    data = select_candidate(planner, q, obj, [0.,-.08,0.,0.], .5)
    assert 'planning_error' in data and 'path' not in data
    assert all(c['rejected_stage'] == 'Approach' for c in data['candidate_report']['candidates'])
    assert 'Approach [nominal, tilted]: IK unreachable' in data['planning_error']
    for report in data['candidate_report']['candidates']:
        assert report['root_reason'] == 'IK unreachable'
        assert 'conveyor intercept IK:' in report['reason_chain'][0]
        assert report['reason_chain'][-1] == 'IK unreachable'


@pytest.mark.parametrize('failed_stage', ['TrackGrasp', 'Lift'])
def test_revalidation_failure_keeps_stage_and_cause(monkeypatch, failed_stage):
    import deux_task_core.grasp_candidates as candidates
    planner, q, obj = fixture()
    def fail(*args, **kwargs):
        raise ValueError('collision clearance')
    if failed_stage == 'TrackGrasp':
        monkeypatch.setattr(candidates, 'plan_track_grasp', fail)
    else:
        monkeypatch.setattr(Planner, 'plan', fail)
    data = revalidate_grasp(planner, 'nominal', q, obj, [0.,-.08,0.,0.], .5)
    assert 'path' not in data
    assert f'{failed_stage} [nominal]: collision clearance' in data['planning_error']
    report = data['candidate_report']['candidates'][0]
    assert report['rejected_stage'] == failed_stage
    assert report['root_reason'] == 'collision clearance'


def test_ranking_rewards_short_motion_and_clearance():
    planner, _, _ = fixture()
    cfg = planner.config['conveyor_tracking']['candidate_search']
    def score(travel, clearance):
        path = np.zeros((2,28));path[1,14] = travel
        return metrics(planner.robot, [dict(path=path, minimum_clearance=clearance)], cfg)['score']
    assert score(1., .2) < score(2., .2)
    assert score(1., .3) < score(1., .1)


def test_retimed_closure_updates_forecast_and_selected_grip_is_reused():
    planner, q, obj = fixture()
    planner.robot.acceleration = np.ones(28)*50
    planner.robot.acceleration[21:28] = .3
    data = select_candidate(planner, q, obj, [0.,-.08,0.,0.], .5)
    report = next(c for c in data['candidate_report']['candidates'] if c['candidate_id'] == data['candidate_id'])
    expected = .5 + report['durations_s']['Approach'] + .4 + report['durations_s']['TrackGrasp']
    assert report['close_forecast_s'] == pytest.approx(expected)
    assert report['close_object_xyz'][1] == pytest.approx(obj[1,3] - .08*expected)
    new = obj.copy();new[0,3] += .01
    refreshed = revalidate_grasp(planner, data['candidate_id'], data['path'][-1], new, [0.,-.08,0.,0.], .5)
    assert refreshed['candidate_id'] == data['candidate_id']
    assert refreshed['path'][-1,14] == pytest.approx(new[0,3])
    assert 'Lift' in refreshed['candidate_report']['ee_paths']
    with pytest.raises(ValueError, match='unknown grasp candidate'):
        revalidate_grasp(planner, 'stale', q, obj, [0.,-.08,0.,0.], .5)


def test_grip_rotation_keeps_same_grasp_point_and_invalid_config_rejected():
    planner, _, _ = fixture()
    validate(planner.config)
    selected = candidate_config(planner.config, 'tilted')
    np.testing.assert_allclose(np.array(selected['grasp_object_from_right_ee'])[:3,3],
                               planner.config['grasp_object_from_right_ee'][:3,3])
    assert not np.allclose(np.array(selected['grasp_object_from_right_ee'])[:3,:3],
                           planner.config['grasp_object_from_right_ee'][:3,:3])
    planner.config['conveyor_tracking']['candidate_search']['candidates'][1]['approach_offset_base_m'][0] = .1
    with pytest.raises(ValueError, match='rendezvous'):validate(planner.config)
