from right_only_trial import planning_robot
from progressive_descent import target as descent_target, next_profile
"""Pure asynchronous upper-motion planning; no ROS or hardware commands.
Every plan retains a stop tail. Replacement retains the original time origin and
all prefix knots through a future splice, so JTC replacement cannot drop motion.
"""
import time
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from deux_task_core.model import RobotModel
from deux_task_core.conveyor import forecast
from deux_task_core.conveyor_screen import clearance
from upper_collision_batch import checked_clearances
from deux_task_core.continuous_grasp_preview import jacobian
from deux_task_core.position_first_preview import desired, bounded_joint_velocity
from deux_task_core.live_continuous_preview import roi_contains


def spline(plan):
    return CubicHermiteSpline(plan['times'], plan['path'], plan['velocities'])


def validate(config, plan, robot=None, check_from=0.):
    r = robot or planning_robot(config)
    t = np.asarray(plan['times']); q = np.asarray(plan['path']); v = np.asarray(plan['velocities'])
    if (len(t)<2 or t[0]!=0 or np.any(np.diff(t)<=0) or q.shape!=(len(t),28)
            or v.shape!=q.shape or not all(np.isfinite(x).all() for x in (t,q,v))):
        raise ValueError('malformed continuous trajectory')
    if np.max(abs(v[-1]))>1e-8: raise ValueError('missing stopped terminal state')
    # Exact polynomial extrema, including both sides of acceleration discontinuities.
    for j in list(range(7))+list(range(14,21)):
        s = CubicHermiteSpline(t,q[:,j],v[:,j])
        reserve=np.deg2rad(float(config.get('intercept_window',{}).get('min_nominal_joint_margin_deg',0.))) if 14<=j<21 else 0.
        for order,low,high in [(0,r.limits[j,0]+reserve,r.limits[j,1]-reserve),(1,-r.velocity[j],r.velocity[j]),(2,-r.acceleration[j],r.acceleration[j])]:
            f=s.derivative(order); roots=f.derivative().roots(extrapolate=False)
            roots=roots[np.isfinite(roots)]; values=f(np.r_[t,np.nextafter(t[1:],t[:-1]),roots])
            if np.min(values)<low-1e-8 or np.max(values)>high+1e-8:
                raise ValueError(f'continuous polynomial limit order={order} joint={j} min={float(np.min(values)):.8f} max={float(np.max(values)):.8f} allowed=[{low:.8f},{high:.8f}] reserve_deg={float(np.degrees(reserve)):.3f}')
    s=spline(plan); minimum=float('inf')
    if not 0<=check_from<t[-1]: raise ValueError('invalid collision validation interval')
    checked=s(np.linspace(check_from,t[-1],int(np.ceil((t[-1]-check_from)/.005))+1))
    minimum=float(np.min(checked_clearances(r,checked,config['obstacles'],config['collision_margin'])))
    return minimum


def eligible(plan, now, target_id, generation):
    return (plan['target_id']==target_id and plan['generation']==generation
            and now < plan['start_at']+plan['splice_s']-.15)


def compute_replacement(config, old, obj, velocity, stamp, splice_s, generation):
    began=time.perf_counter();r=planning_robot(config); s=spline(old)
    if not 0<splice_s<old['brake_start_s']:
        raise ValueError('replacement cannot enter existing brake tail')
    q=np.array(s(splice_s)); qd=np.array(s(splice_s,1))
    initial=np.array(old['initial_pose']); grasp=np.array(old['grasp'])
    old_obj=np.array(old['object_at_start']); old_v=np.array(old['velocity'])
    # Express newest measurement at the fixed trajectory epoch. This is a
    # coordinate-time rebase, not a negative future prediction request.
    measured_velocity=np.asarray(velocity,dtype=float)
    if measured_velocity.shape!=(4,) or not np.isfinite(measured_velocity).all() or abs(measured_velocity[3])>1e-8:
        raise ValueError('rebase requires straight nonrotating belt')
    new_obj=np.array(obj,dtype=float).copy()
    new_obj[:3,3]+=measured_velocity[:3]*(old['start_at']-stamp)
    v=np.array(velocity); horizon=float(config.get('upper_replan_horizon_s',.5))
    if not .2<=horizon<=1.6:raise ValueError('invalid upper replan horizon')
    duration=min(horizon,old['execution_end_s']-splice_s)
    if duration<.2: raise ValueError('trial duration exhausted')
    if not roi_contains(old['geometry'],[forecast(new_obj,v,t)[:3,3] for t in np.linspace(splice_s,splice_s+duration+(0 if config.get('progressive_descent') else 1),12)]):
        raise ValueError('short replacement forecast leaves ROI')
    fixed=old.get('fixed_interception')
    profile=old.get('descent_profile')
    policy=config.get('tracking_grasp_policy')
    follow=old.get('workspace_follow',True)
    admitted=config.get('_descent_admitted',False)
    hold_rotation=old.get('hold_rotation')
    if policy:
        from deux_task_core.tracking_policy import follow_admitted,forecast_inside
        from progressive_descent import height_state
        follow=follow_admitted(policy,new_obj,v,grasp,splice_s+(float(config.get('tracking_lookahead_s',0.)) if profile is None else 0.),previous=follow)
        height,_=height_state(profile,splice_s,config['approach_height'])
        if policy.get('scheduled_descent'):
            from scheduled_descent import profile_for,finish_time
            candidate=profile or profile_for(max(0.,float(config.get('_descent_measured_height',height))),splice_s,policy['descent_speed_m_s'],policy.get('max_descent_m'))
            remaining=finish_time(candidate,splice_s,policy['close_height_m'],policy['close_duration_s'])+float(config.get('_descent_delay_s',0.))
        else:remaining=height/policy['descent_speed_m_s']+policy['close_duration_s']
        if fixed is not None and profile is None and admitted:
            from fixed_interception import schedule
            candidate=schedule(fixed,new_obj,v,grasp,splice_s,max(0.,float(config.get('_descent_measured_height',height))),policy)
            remaining=finish_time(candidate,splice_s,policy['close_height_m'],policy['close_duration_s'])
        admitted=admitted and follow and forecast_inside(policy,new_obj,v,grasp,splice_s,splice_s+remaining,policy['entry_margin_m'])
        if profile is not None and not admitted:raise ValueError('descent admission lost; retain checked stop tail')
        if height<=policy['rotation_freeze_height_m'] and hold_rotation is None:
            hold_rotation=r.ee_fk('right',q)[:3,:3].tolist()
    if config.get('progressive_descent',False):
        if policy and policy.get('scheduled_descent'):
            if profile is None and admitted:
                profile=candidate
                hold_rotation=r.ee_fk('right',q)[:3,:3].tolist()
        elif splice_s>=old['approach_s']:
            profile=next_profile(old,splice_s,duration,config['approach_height'],admitted,speed=policy['descent_speed_m_s'] if policy else .06)
    # Workspace tracking uses its own XY box and image ROI, not the later
    # fixed interception polygon. Non-policy interception keeps that gate.
    if config.get('intercept_window') and profile is not None and not policy:
        from intercept_window import contains
        if not contains(config['intercept_window'],[forecast(new_obj,v,t)[:3,3] for t in np.linspace(splice_s,splice_s+duration,10)]):
            raise ValueError('descent segment leaves grasp window')
    if policy and profile is not None:
        h,_=height_state(profile,splice_s+duration,config['approach_height'])
        finish=(splice_s+duration+finish_time(profile,splice_s+duration,policy['close_height_m'],policy['close_duration_s'])+float(config.get('_descent_delay_s',0.)) if policy.get('scheduled_descent') else splice_s+duration+h/policy['descent_speed_m_s']+policy['close_duration_s'])
        if not forecast_inside(policy,new_obj,v,grasp,splice_s,finish):raise ValueError('predicted close leaves XY workspace')
    if fixed is None and profile is None and not roi_contains(old['geometry'],[forecast(new_obj,v,t+float(config.get('tracking_lookahead_s',0.)))[:3,3] for t in np.linspace(splice_s,splice_s+duration,12)]):
        raise ValueError('upper lookahead leaves ROI')
    prepared=time.perf_counter()
    ts=np.linspace(splice_s,splice_s+duration,int(np.ceil(duration/.02))+1)
    path=[q.copy()]; velocities=[qd.copy()]; residuals=[]
    for a,b in zip(ts[:-1],ts[1:]):
        h=b-a
        before,old_feed=descent_target(initial,old_obj,old_v,grasp,config['approach_height'],old['approach_s'],a,old.get('descent_profile'),lookahead=float(config.get('tracking_lookahead_s',1.)),workspace=policy,workspace_follow=old.get('workspace_follow',True),hold_rotation=old.get('hold_rotation'))
        after,feed=descent_target(initial,new_obj,v,grasp,config['approach_height'],old['approach_s'],a,profile,lookahead=float(config.get('tracking_lookahead_s',1.)),workspace=policy,workspace_follow=follow,hold_rotation=hold_rotation)
        if fixed is not None:
            from fixed_interception import target as fixed_target
            before,old_feed=fixed_target(initial,fixed,config['approach_height'],old['approach_s'],a,old.get('descent_profile'),old.get('hold_rotation'))
            after,feed=fixed_target(initial,fixed,config['approach_height'],old['approach_s'],a,profile,hold_rotation)
        # Bounded correction, C1 ramp: retain old target at splice, absorb latest evidence over .5s.
        u=min(1.,(a-splice_s)/.5); blend=u*u*(3-2*u); rate=6*u*(1-u)/.5
        target=after.copy();delta=after[:3,3]-before[:3,3]
        target[:3,3]=before[:3,3]+blend*delta
        rot=Rotation.from_matrix(after[:3,:3]@before[:3,:3].T).as_rotvec()
        target[:3,:3]=Rotation.from_rotvec(blend*rot).as_matrix()@before[:3,:3]
        new_feed=feed.copy()
        feed=(1-blend)*old_feed+blend*feed+rate*np.r_[delta,rot]
        if profile is not None and a>=profile['start']:
            target[2,3]=after[2,3];feed[2]=new_feed[2]
        fk,j=jacobian(r,q)
        error=np.r_[target[:3,3]-fk[:3,3],Rotation.from_matrix(target[:3,:3]@fk[:3,:3].T).as_rotvec()]
        if not np.isfinite(error).all():raise ValueError('nonfinite Cartesian residual')
        residuals.append((float(np.linalg.norm(error[:3])),float(np.linalg.norm(error[3:]))))
        weights=np.array([1,1,1,*([.05]*3 if config.get('progressive_descent') else [.2]*3)]); jw=j*weights[:,None]
        task=feed+np.r_[6*error[:3],4*error[3:]]
        wanted=np.linalg.solve(jw.T@jw+.015**2*np.eye(7),jw.T@(weights*task))
        wanted=bounded_joint_velocity(r,q,wanted)
        next_v=qd.copy();next_v[14:21]+=np.clip(wanted-qd[14:21],-.95*r.acceleration[14:21]*h,.95*r.acceleration[14:21]*h)
        q=q+(qd+next_v)*h/2;qd=next_v;path.append(q.copy());velocities.append(qd.copy())
    ik_finished=time.perf_counter()
    brake=max(.6,float(np.max(abs(qd)/(.7*r.acceleration))))
    bt=np.linspace(0,brake,int(np.ceil(brake/.02))+1)[1:]
    path=np.vstack([path,q+qd*(bt-bt*bt/(2*brake))[:,None]])
    velocities=np.vstack([velocities,qd*(1-bt/brake)[:,None]])
    brake_start=ts[-1];ts=np.r_[ts,brake_start+bt]
    # Coalesce a splice at an existing knot; sub-nanosecond intervals amplify
    # roundoff into artificial cubic acceleration spikes.
    keep=np.array(old['times'])<splice_s-1e-9
    result=dict(old,times=np.r_[np.array(old['times'])[keep],ts].tolist(),
                path=np.vstack([np.array(old['path'])[keep],path]).tolist(),
                velocities=np.vstack([np.array(old['velocities'])[keep],velocities]).tolist(),
                splice_s=splice_s,generation=generation,brake_start_s=float(brake_start),
                object_at_start=new_obj.tolist(),velocity=v.tolist(),source_object=np.array(obj).tolist(),source_stamp=stamp,descent_profile=profile,workspace_follow=follow,hold_rotation=hold_rotation)
    if not roi_contains(old['geometry'],[forecast(new_obj,v,t)[:3,3] for t in np.linspace(splice_s,ts[-1]+(0 if config.get('progressive_descent') else 1),20)]):
        raise ValueError('replacement forecast leaves ROI')
    result['cartesian_residual']=dict(policy='warning_only',position_m=max(x[0] for x in residuals),rotation_rad=max(x[1] for x in residuals))
    validation_began=time.perf_counter()
    result['min_conveyor_margin_m']=validate(config,result,r,check_from=splice_s)
    if policy and policy.get('scheduled_descent') and profile is not None:
        from descent_timing import validated_timing
        result['descent_timing']=validated_timing(r,result,policy)
        completion=result['descent_timing'].get('completion_s')
        if completion is not None:
            end=max(splice_s,completion+float(config.get('_descent_delay_s',0.)))
            if not forecast_inside(policy,new_obj,v,grasp,splice_s,end):raise ValueError('pose-timed closure leaves workspace')
            if not roi_contains(old['geometry'],[forecast(new_obj,v,t)[:3,3] for t in np.linspace(splice_s,end,12)]):raise ValueError('pose-timed closure leaves ROI')
    finished=time.perf_counter()
    result['updated_window_s']=duration
    result['planning_timing_ms']=dict(prepare=(prepared-began)*1000,ik=(ik_finished-prepared)*1000,assemble=(validation_began-ik_finished)*1000,validation=(finished-validation_began)*1000,total=(finished-began)*1000)
    return result
