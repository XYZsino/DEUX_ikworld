"""Batched time × grip screening followed by bounded full-chain validation.

Times are relative to the SAME acquisition snapshot. A selected arrival/close
is never silently retimed during fresh validation. No hardware-limit changes.
"""
from types import SimpleNamespace
import time
import numpy as np
from .model import pose, vector, HAND_NAMES, JOINT_NAMES
from .conveyor import forecast, plan_conveyor, plan_track_grasp


def settings(config):
    return config.get('conveyor_tracking',{}).get('candidate_search',{}).get('time_search',{})


def time_grid(cfg):
    return np.arange(cfg['arrival_min_s'],cfg['arrival_max_s']+cfg['step_s']*.1,cfg['step_s'])


def validate_time_search(config):
    cfg=settings(config)
    if not cfg.get('enabled',False):return
    from .planning_latency import settings as latency_settings
    latency_settings(config)
    if not config['conveyor_tracking']['candidate_search'].get('enabled'):raise ValueError('time search requires candidates')
    for key in ('arrival_min_s','arrival_max_s','step_s','handoff_s','max_prediction_std_m'):
        if not np.isfinite(cfg[key]) or cfg[key]<=0:raise ValueError('invalid time search '+key)
    if cfg['arrival_min_s']>cfg['arrival_max_s'] or len(time_grid(cfg))>25:raise ValueError('invalid time grid')
    if not isinstance(cfg['full_path_candidates'],int) or not 1<=cfg['full_path_candidates']<=12:raise ValueError('invalid full path shortlist')
    minimum=.25 if cfg.get('prefetch_grasp',False) else config['conveyor_tracking']['start_lead']
    if cfg['handoff_s']<minimum:raise ValueError('handoff shorter than launch lead')


def batch_ik(robot, targets, seeds):
    if getattr(robot,'ik_solver',None) is not None:
        q, errors=robot.ik_solver.solve_batch('right',targets,seeds,.005)
        return q,errors,'native_batch'
    # Explicit serial compatibility for SciPy and synthetic fixtures.
    output=np.asarray(seeds).copy();errors=[]
    for i,(target,seed) in enumerate(zip(targets,seeds)):
        try:output[i]=robot.ik('right',target,seed,continuity_weight=.005);errors.append('')
        except ValueError as error:errors.append(str(error))
    return output,errors,'serial_compatibility'


def predicted_std(covariance, horizon, config):
    if covariance is None:return None
    cov=np.asarray(covariance,dtype=float).reshape(8,8)
    if not np.isfinite(cov).all() or not np.allclose(cov,cov.T,atol=1e-6) or np.linalg.eigvalsh(cov).min() < -1e-8:
        raise ValueError('invalid motion covariance')
    f=np.eye(8);f[:4,4:]=np.eye(4)*horizon
    cfg=config['conveyor_tracking']
    if cfg.get('occluded_grasp_enabled',False):
        # Match conveyor_velocity/validate_occlusion_prediction: after the
        # measured-motion agreement gate, the belt is an independent prior.
        projected=cov[:3,:3]+np.eye(3)*(cfg['belt_velocity_std_m_s']*horizon)**2
    else:projected=(f@cov@f.T)[:3,:3]
    process=cfg['prediction_acceleration_std']
    return float(np.sqrt(max(0.,np.linalg.eigvalsh(projected).max())+process**2*horizon**4/4))


def full_chain(planner,start,target,velocity,lead,identity,arrival_lead,grasp_lead,phase_durations=None):
    from .grasp_candidates import candidate_config,metrics,settings as candidate_settings,preview,rejection
    from .planning import Planner
    cfg=candidate_config(planner.config,identity)
    duration=arrival_lead-lead
    if duration<=0:raise ValueError('arrival deadline elapsed')
    cfg['conveyor_tracking'].update(fixed_approach_duration_s=duration,intercept_wait_s=grasp_lead-arrival_lead)
    if phase_durations is not None:cfg['conveyor_tracking']['fixed_grasp_durations_s']=phase_durations
    trial=SimpleNamespace(config=cfg,robot=planner.robot);stage='Approach'
    report=dict(candidate_id=identity,feasible=False,arrival_lead_s=float(arrival_lead),grasp_start_lead_s=float(grasp_lead))
    try:
        approach=plan_conveyor(trial,'Approach',start,target,velocity,lead)
        stage='TrackGrasp';grasp=plan_track_grasp(trial,approach['path'][-1],target,velocity,grasp_lead)
        stage='Lift';lift=Planner.plan(trial,'Lift',grasp['path'][-1],np.eye(4))
        plans=dict(Approach=approach,TrackGrasp=grasp,Lift=lift)
        phases=np.diff(np.r_[0.,grasp['phase_end_times']]).tolist()
        report.update(feasible=True,**metrics(planner.robot,plans.values(),candidate_settings(planner.config)),
                      close_lead_s=float(grasp_lead+grasp['times'][-1]),phase_durations=phases,
                      durations_s={k:float(p['times'][-1]) for k,p in plans.items()},ee_paths=preview(planner.robot,plans))
        return approach,report
    except ValueError as error:
        report.update(rejection(stage,error));return None,report


def search(planner,start,target,velocity,lead,covariance=None):
    from .grasp_candidates import candidate_config,settings as candidate_settings
    tick=time.perf_counter();cfg=settings(planner.config);base=candidate_settings(planner.config)
    q=vector(start,28);target=pose(target);velocity=vector(velocity,4)
    rows=[];approach_targets=[];close_targets=[];lift_targets=[]
    hand_indices=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    from .grasp_timing import initial_durations
    initial_phases=initial_durations(planner.config,planner.robot)
    grasp_duration=float(sum(initial_phases))
    for duration in time_grid(cfg):
        for item in base['candidates']:
            chosen=candidate_config(planner.config,item['id']);arrival=lead+duration;grasp_start=arrival+cfg['handoff_s'];close=grasp_start+grasp_duration
            approach=forecast(target,velocity,grasp_start)@pose(chosen['grasp_object_from_right_ee'])
            approach[2,3]+=chosen['approach_height'];approach[:3,3]+=item['approach_offset_base_m']
            grip=forecast(target,velocity,close)@pose(chosen['grasp_object_from_right_ee'])
            lift=grip.copy();lift[2,3]+=chosen['lift_height']
            std=predicted_std(covariance,close,planner.config)
            reason=''
            if close>planner.config['conveyor_tracking']['max_plan_horizon']:reason='prediction horizon exceeded'
            elif std is not None and std>cfg['max_prediction_std_m']:reason='prediction uncertainty exceeds bound'
            rows.append(dict(candidate_id=item['id'],yaw_deg=float(item['yaw_deg']),arrival_after_start_s=float(duration),arrival_lead_s=float(arrival),
                grasp_start_lead_s=float(grasp_start),representative_close_lead_s=float(close),initial_phase_durations=initial_phases.tolist(),prediction_std_m=std,
                screening_passed=False,full_path_checked=False,feasible=False,rejected_stage='Prediction' if reason else '',reason=reason))
            approach_targets.append(approach);close_targets.append(grip);lift_targets.append(lift)
    seeds=np.repeat(q[None],len(rows),axis=0)
    clearance=np.full(len(rows),np.inf);mode=''
    for stage,targets in [('Approach',approach_targets),('Close',close_targets),('Lift',lift_targets)]:
        previous=seeds.copy()
        seeds,errors,mode=batch_ik(planner.robot,targets,seeds)
        seeds[:,hand_indices]=planner.config.get('hand_pregrasp',planner.config['hand_open']) if stage=='Approach' else planner.config['hand_closed']
        for i,row in enumerate(rows):
            if row['reason']:continue
            row['representative_travel_rad']=row.get('representative_travel_rad',0.)+float(np.linalg.norm((seeds[i]-previous[i])[14:21]))
            reason=errors[i]
            if not reason:
                try:clearance[i]=min(clearance[i],planner.robot.clearance(seeds[i],planner.config['obstacles'],planner.config['collision_margin'],planner.config['attached_radius'] if stage=='Lift' else 0.))
                except ValueError as error:reason=str(error)
            if stage=='Approach' and not reason:
                from .conveyor import joint_approach_path
                _,_,factor=joint_approach_path(planner.robot,q,seeds[i],planner.config.get('hand_pregrasp',planner.config['hand_open']),row['arrival_after_start_s'],planner.config['sample_period'])
                row['approach_dynamics_ratio']=factor
                if factor>.98:reason='arrival time infeasible under joint dynamics'
                row['joint_travel_rad']=float(np.linalg.norm((seeds[i]-q)[14:21]))
            if reason:row.update(rejected_stage=stage+'Screen',reason=reason)
    for i,row in enumerate(rows):
        if not row['reason']:
            row.update(screening_passed=True,minimum_clearance_m=float(clearance[i]))
            row['screen_score']=row['representative_travel_rad']-2*min(clearance[i]/base['clearance_scale_m'],1.)
            ratio=row['approach_dynamics_ratio']
            row['screen_score']+=.05*ratio/(1.05-ratio)
    # Reward adjacent feasible time bins, then bound expensive validation.
    for row in rows:
        if row['screening_passed']:
            neighbours=sum(other['screening_passed'] and other['candidate_id']==row['candidate_id'] and abs(other['arrival_after_start_s']-row['arrival_after_start_s'])<=cfg['step_s']*1.01 for other in rows)-1
            row['adjacent_time_bins']=neighbours;row['screen_score']-=.05*neighbours
    ranked=sorted((r for r in rows if r['screening_passed']),key=lambda r:(r['screen_score'],r['arrival_after_start_s'],r['candidate_id']))
    # Small Cartesian offset variants must not consume the entire shortlist.
    # First retain the best time per grip yaw, then fill remaining slots.
    shortlist=[];yaw_seen=set()
    for row in ranked:
        if row['yaw_deg'] not in yaw_seen:
            shortlist.append(row);yaw_seen.add(row['yaw_deg'])
    shortlist=shortlist[:cfg['full_path_candidates']]
    for row in ranked:
        if len(shortlist)>=cfg['full_path_candidates']:break
        if not any(row is selected for selected in shortlist):shortlist.append(row)
    good=[]
    for row in shortlist:
        plan,report=full_chain(planner,q,target,velocity,lead,row['candidate_id'],row['arrival_lead_s'],row['grasp_start_lead_s'])
        if report['feasible']:
            std=predicted_std(covariance,report['close_lead_s'],planner.config)
            if std is not None and std>cfg['max_prediction_std_m']:
                report.update(feasible=False,rejected_stage='Prediction',reason='retimed closure uncertainty exceeds bound');plan=None
        row.update(report,full_path_checked=True)
        if plan is not None:good.append((row['score'],row['arrival_after_start_s'],row['candidate_id'],plan,row))
    diagnostics=dict(scope='time × grip batch screening + bounded full-chain validation',time_search=True,
        batch_mode=mode,time_bins_s=time_grid(cfg).tolist(),candidates=rows,selected_candidate_id='',
        screened_count=len(rows),screen_passed=sum(r['screening_passed'] for r in rows),full_path_count=len(shortlist),elapsed_s=time.perf_counter()-tick)
    if not good:return dict(planning_error=f'no feasible timed grasp: {diagnostics["screen_passed"]}/{len(rows)} screened; {len(shortlist)} full paths rejected',candidate_report=diagnostics)
    _,_,identity,plan,selected=min(good,key=lambda x:x[:3])
    timing=dict(arrival_lead_s=selected['arrival_lead_s'],grasp_start_lead_s=selected['grasp_start_lead_s'],phase_durations=selected['phase_durations'])
    diagnostics.update(selected_candidate_id=identity,selected_arrival_after_start_s=selected['arrival_after_start_s'],selected_timing=timing)
    return dict(plan,candidate_id=identity,candidate_report=diagnostics,ik_calls=planner.robot.ik_calls,timing=timing)


def revalidate(planner,start,target,velocity,lead,identity,timing,covariance=None):
    plan,report=full_chain(planner,start,target,velocity,lead,identity,timing['arrival_lead_s'],timing['grasp_start_lead_s'],timing['phase_durations'])
    if plan is not None:
        std=predicted_std(covariance,report['close_lead_s'],planner.config)
        if std is not None and std>settings(planner.config)['max_prediction_std_m']:
            report.update(feasible=False,rejected_stage='Prediction',reason='closure uncertainty exceeds bound');plan=None
    diagnostics=dict(scope='fixed-time fresh full-chain revalidation',time_search=True,candidates=[report],selected_candidate_id=identity,selected_timing=timing)
    if plan is None:return dict(planning_error=f'timed grasp revalidation: {report["rejected_stage"]}: {report["reason"]}',candidate_report=diagnostics)
    return dict(plan,candidate_id=identity,candidate_report=diagnostics,ik_calls=planner.robot.ik_calls,timing=timing)
