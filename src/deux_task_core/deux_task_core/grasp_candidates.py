"""Bounded moving-grasp search. Predictions are plans, never grasp evidence."""
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from .model import pose, vector
from .conveyor import plan_conveyor, plan_track_grasp, forecast


def settings(config):
    return config.get('conveyor_tracking', {}).get('candidate_search', {})


def rejection(stage, error):
    # Preserve wrapped IK errors: the outer exception contains the target,
    # while its cause contains the actual residual or joint-limit failure.
    chain = []
    seen = set()
    while error is not None and id(error) not in seen:
        seen.add(id(error))
        chain.append(str(error))
        error = error.__cause__
    return dict(rejected_stage=stage, reason=chain[0], root_reason=chain[-1],
                reason_chain=chain)


def failure_summary(reports):
    groups = {}
    for item in reports:
        if not item.get('feasible', False):
            key = (item['rejected_stage'], item.get('root_reason', item['reason']))
            groups.setdefault(key, []).append(item['candidate_id'])
    return '\n'.join(f"{stage} [{', '.join(ids)}]: {reason}"
                     for (stage, reason), ids in groups.items())


def validate(config):
    cfg = settings(config)
    if not cfg.get('enabled', False):
        return
    from .time_candidates import validate_time_search
    validate_time_search(config)
    candidates = cfg['candidates']
    if not 1 <= len(candidates) <= 12:
        raise ValueError('invalid grasp candidate count')
    ids = set()
    for item in candidates:
        identity = item['id']
        if not isinstance(identity, str) or not identity or identity in ids:
            raise ValueError('invalid grasp candidate ID')
        ids.add(identity)
        if not np.isfinite(item['yaw_deg']) or abs(item['yaw_deg']) > 30:
            raise ValueError('invalid grasp candidate yaw')
        offset = vector(item['approach_offset_base_m'], 3)
        # Same-height nearby approaches preserve the measured rendezvous gate.
        if abs(offset[2]) > 1e-9 or np.linalg.norm(offset[:2]) >= config['conveyor_tracking']['intercept_trigger_distance_m']:
            raise ValueError('candidate approach must fit rendezvous tolerance')
    for key in ('selection_start_lead_s', 'joint_travel_weight', 'clearance_weight', 'clearance_scale_m'):
        if not np.isfinite(cfg[key]) or cfg[key] <= 0:
            raise ValueError('invalid candidate search setting: ' + key)
    if cfg['selection_start_lead_s'] < config['conveyor_tracking']['start_lead']:
        raise ValueError('candidate search lead shorter than launch lead')


def candidate_config(config, identity):
    cfg = settings(config)
    item = next((c for c in cfg.get('candidates', []) if c['id'] == identity), None)
    if item is None:
        raise ValueError('unknown grasp candidate: ' + identity)
    out = deepcopy(config)
    transform = pose(config['grasp_object_from_right_ee']).copy()
    # Rotate the grip at the same grasp point; do not orbit its translation.
    transform[:3, :3] = Rotation.from_euler('z', item['yaw_deg'], degrees=True).as_matrix() @ transform[:3, :3]
    out['grasp_object_from_right_ee'] = transform.tolist()
    out['conveyor_tracking']['approach_offset_base_m'] = item['approach_offset_base_m']
    return out


def metrics(robot, plans, cfg):
    travel = sum(float(np.linalg.norm(np.diff(p['path'][:, 14:21], axis=0), axis=1).sum()) for p in plans)
    clearance = min(float(p['minimum_clearance']) for p in plans)
    # Dimensionless clearance reward saturates rather than overpowering travel.
    score = cfg['joint_travel_weight'] * travel - cfg['clearance_weight'] * min(clearance / cfg['clearance_scale_m'], 1.)
    return dict(joint_travel_rad=travel, minimum_clearance_m=clearance, score=float(score))


def preview(robot, plans):
    return {name: [robot.fk(q)[robot.ee['right']][:3, 3].tolist()
                   for q in data['path'][np.unique(np.linspace(0, len(data['path'])-1, 24).astype(int))]]
            for name, data in plans.items()}


def select_candidate(planner, start, target, velocity, lead, identity=''):
    """Evaluate every candidate through Lift at retimed conveyor timestamps."""
    from .planning import Planner
    cfg = settings(planner.config)
    reports = []
    feasible = []
    choices=cfg['candidates']
    if identity:
        candidate_config(planner.config,identity)  # Reject unknown/stale IDs.
        choices=[item for item in choices if item['id']==identity]
    for item in choices:
        report = dict(candidate_id=item['id'], yaw_deg=item['yaw_deg'],
                      approach_offset_base_m=item['approach_offset_base_m'], feasible=False)
        reports.append(report)
        selected = candidate_config(planner.config, item['id'])
        # Reuse the immutable robot model; IK counters are cumulative.
        trial = SimpleNamespace(config=selected, robot=planner.robot)
        stage = 'Approach'
        try:
            approach = plan_conveyor(trial, stage, start, target, velocity, lead)
            # Approach aims at this future rendezvous. Each underlying planner
            # repeats IK against the new forecast whenever dynamics retime it.
            grasp_lead = lead + float(approach['times'][-1]) + selected['conveyor_tracking'].get('intercept_wait_s', 0.)
            stage = 'TrackGrasp'
            grasp = plan_track_grasp(trial, approach['path'][-1], target, velocity, grasp_lead)
            stage = 'Lift'
            lift = Planner.plan(trial, stage, grasp['path'][-1], np.eye(4))
            plans = dict(Approach=approach, TrackGrasp=grasp, Lift=lift)
            report.update(feasible=True, **metrics(planner.robot, plans.values(), cfg),
                          durations_s={name: float(p['times'][-1]) for name, p in plans.items()},
                          close_forecast_s=float(grasp_lead + grasp['times'][-1]),
                          close_object_xyz=forecast(target, velocity, grasp_lead + grasp['times'][-1])[:3, 3].tolist(),
                          ee_paths=preview(planner.robot, plans))
            feasible.append((report['score'], item['id'], approach))
        except ValueError as error:
            report.update(rejection(stage, error))
    diagnostics = dict(candidates=reports, selected_candidate_id='',
                       scope=('fresh selected Approach + TrackGrasp + Lift revalidation' if identity
                              else 'Approach + TrackGrasp + Lift candidate comparison'))
    if not feasible:
        return dict(planning_error='no feasible grasp candidate through Lift\n' + failure_summary(reports),
                    candidate_report=diagnostics)
    _, identity, approach = min(feasible, key=lambda item: (item[0], item[1]))
    diagnostics['selected_candidate_id'] = identity
    return dict(approach, candidate_id=identity, candidate_report=diagnostics, ik_calls=planner.robot.ik_calls)


def revalidate_grasp(planner, identity, start, target, velocity, lead, timing=None):
    """Keep the selected grip, but recompute from the NEW measured snapshot."""
    from .planning import Planner
    selected = candidate_config(planner.config, identity)
    if timing is not None:selected['conveyor_tracking']['fixed_grasp_durations_s']=timing['phase_durations']
    trial = SimpleNamespace(config=selected, robot=planner.robot)
    stage = 'TrackGrasp'
    try:
        grasp = plan_track_grasp(trial, start, target, velocity, lead)
        stage = 'Lift'
        lift = Planner.plan(trial, stage, grasp['path'][-1], np.eye(4))
    except ValueError as error:
        rejected = dict(candidate_id=identity, feasible=False, **rejection(stage, error))
        return dict(planning_error='grasp candidate revalidation failed\n' + failure_summary([rejected]),
                    candidate_report=dict(selected_candidate_id=identity,
                        scope='fresh TrackGrasp + Lift revalidation', candidates=[rejected]))
    report = dict(selected_candidate_id=identity, scope='fresh TrackGrasp + Lift revalidation',
                  **metrics(planner.robot, (grasp, lift), settings(planner.config)),
                  close_forecast_s=float(lead + grasp['times'][-1]),
                  ee_paths=preview(planner.robot, dict(TrackGrasp=grasp, Lift=lift)))
    return dict(grasp, candidate_id=identity, candidate_report=report, ik_calls=planner.robot.ik_calls)
