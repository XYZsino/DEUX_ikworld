"""Time-consistent, bounded interception using the existing trajectory executor.

The zone is a task constraint, not a statement that every pose is reachable.
Every candidate also needs an approach/descent/closure/brake feasibility check.
"""
import copy,json,time
import numpy as np
from scipy.spatial.transform import Rotation
from deux_task_core.conveyor import forecast
from deux_task_core.position_first_preview import integrate
from deux_task_core.continuous_grasp_preview import plan as descent_plan
from deux_task_core.conveyor_screen import point_distances,surfaces,WRIST_GRASP_RADIUS_M,EXTRA_MARGIN_M
from deux_task_core.live_continuous_preview import roi_contains
from right_only_trial import planning_robot
from types import SimpleNamespace
from progressive_descent import CLOSE_POSITION_TOLERANCE_M

def validate_zone(z):
    origin=np.asarray(z['origin_base_m'],float);axis=np.asarray(z['direction_xy'],float)
    along=np.asarray(z['along_m'],float);cross=float(z['half_width_m']);height=np.asarray(z['z_range_m'],float)
    if origin.shape!=(3,) or axis.shape!=(2,) or along.shape!=(2,) or height.shape!=(2,):
        raise ValueError('invalid intercept zone shape')
    if not np.isfinite(np.r_[origin,axis,along,cross,height]).all() or abs(np.linalg.norm(axis)-1)>1e-6:
        raise ValueError('invalid intercept zone values')
    if along[0]>=along[1] or height[0]>=height[1] or cross<=0:raise ValueError('empty intercept zone')
    return origin,axis,along,cross,height

def coordinates(zone,points):
    origin,axis,_,_,_=validate_zone(zone)
    points=np.asarray(points,float)
    return np.stack([(points[...,:2]-origin[:2])@axis,(points[...,:2]-origin[:2])@np.array([-axis[1],axis[0]]),points[...,2]],axis=-1)

def contains(zone,points):
    p=coordinates(zone,points);_,_,along,width,z=validate_zone(zone)
    return bool(np.all(np.isfinite(p)) and np.all(p[...,0]>=along[0]-1e-9) and np.all(p[...,0]<=along[1]+1e-9)
                and np.all(abs(p[...,1])<=width+1e-9) and (not zone.get('enforce_height_range',True) or (np.all(p[...,2]>=z[0]-1e-9) and np.all(p[...,2]<=z[1]+1e-9))))

def polygon(zone):
    origin,axis,along,width,_=validate_zone(zone);normal=np.array([-axis[1],axis[0]])
    return [np.r_[origin[:2]+axis*s+normal*n,origin[2]].tolist() for s,n in [(along[0],-width),(along[1],-width),(along[1],width),(along[0],width)]]

def arrival_times(zone,obj,v,close_duration,min_approach=.8,max_approach=4.):
    # Interception time is relative to scheduled trajectory start.
    p=coordinates(zone,np.asarray(obj)[:3,3]);origin,axis,along,width,z=validate_zone(zone)
    v=np.asarray(v);speed=float(v[:2]@axis)
    if speed<.01 or np.linalg.norm(v[:2]-speed*axis)>.04:
        raise ValueError('belt direction/speed outside intercept model')
    enter=(along[0]-p[0])/speed;leave=(along[1]-p[0])/speed-close_duration
    lo=max(min_approach,enter);hi=min(max_approach,leave)
    if hi<lo:raise ValueError('no time to intercept and close inside grasp window')
    times=np.linspace(lo,hi,3) if hi-lo>.1 else np.array([lo])
    return [float(t) for t in times if contains(zone,[forecast(obj,v,u)[:3,3] for u in [t,t+close_duration]])]

def add_brake(r,t,path,vel):
    t=np.asarray(t);path=np.asarray(path);vel=np.asarray(vel);qd=vel[-1]
    duration=max(.6,float(np.max(abs(qd)/(.7*r.acceleration))))
    bt=np.linspace(0,duration,int(np.ceil(duration/.02))+1)[1:]
    return np.r_[t,t[-1]+bt],np.vstack([path,path[-1]+qd*(bt-bt*bt/(2*duration))[:,None]]),np.vstack([vel,qd*(1-bt/duration)[:,None]]),duration

def compute_intercept(config,q,obj,v,advance,geometry):
    from async_upper import validate
    began=time.perf_counter();c=copy.deepcopy(config);r=planning_robot(c)
    q=np.asarray(q,float);obj=forecast(np.asarray(obj,float),np.asarray(v,float),advance);v=np.asarray(v,float)
    zone=c['intercept_window'];validate_zone(zone)
    initial=r.ee_fk('right',q)
    descent_s=max(1.,float(c['approach_height'])/.06);duration=descent_s+.5
    times=(list(c['tracking_grasp_policy']['approach_times_s']) if c.get('tracking_grasp_policy') else arrival_times(zone,obj,v,0. if c.get('intercept_approach_first') else duration))
    if not times:raise ValueError('intercept window unavailable')
    g=np.asarray(c['grasp_object_from_right_ee']).copy()
    yaw=float(np.degrees(np.arctan2(obj[1,0],obj[0,0]))%180)
    g[:3,:3]=Rotation.from_euler('z',np.clip(yaw,30,150)-yaw,degrees=True).as_matrix()@g[:3,:3]
    candidates=[]
    for flip in [0,180]:
        grasp=g.copy();grasp[:3,:3]=Rotation.from_euler('z',flip,degrees=True).as_matrix()@g[:3,:3]
        angle=Rotation.from_matrix((obj@grasp)[:3,:3]@initial[:3,:3].T).magnitude()
        candidates.append((float(angle),flip,grasp))
    candidates.sort(key=lambda row:row[0]);failures=[]
    if c.get('intercept_approach_first'):
        for approach in times:
            for angle,flip,grasp in candidates:
                candidate_config=dict(c,grasp_object_from_right_ee=grasp.tolist())
                if c.get('tracking_grasp_policy'):
                    from deux_task_core.tracking_policy import follow_admitted
                    candidate_config['_workspace_follow']=follow_admitted(c['tracking_grasp_policy'],obj,v,grasp,0.)
                    result,fail=policy_upper(r,candidate_config,q,obj,v,grasp,approach,duration,initial,geometry,flip)
                    failures.extend(fail)
                    if result is not None:return result
                    continue
                # Only the executed upper segment is admitted here. Each descent
                # replacement is independently checked before controller dispatch.
                upper=integrate(r,candidate_config,q,obj,v,grasp,approach,10.,tracking_window_s=.8,residual_warning_only=True,check_conveyor=False)
                if not upper['valid']:
                    failures.append(dict(stage='upper',reason=upper['reason']));continue
                ts,path,vel,brake=add_brake(r,upper['times'],upper['path'],upper['velocities'])
                result=dict(times=ts.tolist(),path=path.tolist(),velocities=vel.tolist(),
                    initial_pose=initial.tolist(),grasp=grasp.tolist(),brake_start_s=float(upper['times'][-1]),
                    execution_end_s=min(10.,approach+duration+1.5),approach_s=approach,tracking_s=.8,
                    braking_s=brake,flip=flip,object_at_start=obj.tolist(),velocity=v.tolist(),
                    geometry=geometry,tracking_lookahead_s=0.)
                if not roi_contains(geometry,[forecast(obj,v,t)[:3,3] for t in np.linspace(0,approach+.8,20)]):
                    failures.append('upper tracking forecast leaves observation ROI');continue
                try:margin=validate(c,result,r)
                except ValueError as e:failures.append(dict(stage='upper',reason=str(e)));continue
                result['intercept']=dict(approach_s=approach,object_position=forecast(obj,v,approach)[:3,3].tolist(),
                    full_path_validated=False,upper_path_validated=True,descent_policy='validate_live_replacement',
                    nominal_descent_s=descent_s,min_conveyor_margin_m=margin,zone=zone,
                    live_close_tolerance_m=CLOSE_POSITION_TOLERANCE_M,planning_ms=(time.perf_counter()-began)*1000)
                result['cartesian_residual']=dict(policy='warning_only',position_m=upper['max_position_error_m'],rotation_rad=upper['max_rotation_error_rad'])
                return result
        raise ValueError('no feasible checked upper approach: '+json.dumps(failures))

    for approach in times:
        closure_point=forecast(obj,v,approach+duration)[:3,3]
        if not roi_contains(geometry,[forecast(obj,v,t)[:3,3] for t in np.linspace(approach,approach+duration,20)]):
            failures.append('nominal descent/closure leaves observation ROI');continue
        for angle,flip,grasp in candidates:
            contact_points=np.array([(forecast(obj,v,t)@grasp)[:3,3] for t in np.linspace(approach+descent_s,approach+duration,8)])
            # Necessary lower bound: even a allowed closure position error cannot clear this envelope.
            required=WRIST_GRASP_RADIUS_M+EXTRA_MARGIN_M+surfaces()[4]
            best_possible=point_distances(contact_points)+CLOSE_POSITION_TOLERANCE_M-required
            if np.any(best_possible<0):
                failures.append(dict(reason='grasp endpoint inside conveyor envelope even with 5cm position tolerance',approach_s=approach,flip=flip,best_possible_margin_m=float(best_possible.min())))
                continue
            candidate_config=dict(c,grasp_object_from_right_ee=grasp.tolist())
            a=integrate(r,candidate_config,q,obj,v,grasp,approach,10.,tracking_window_s=0.,residual_warning_only=True)
            if not a['valid']:failures.append(dict(reason=a['reason'],stage='approach',approach_s=approach,flip=flip));continue
            at_obj=forecast(obj,v,approach)
            d=descent_plan(SimpleNamespace(robot=r,config=candidate_config),np.asarray(a['path'][-1]),np.asarray(a['velocities'][-1]),at_obj,v,descent_s,.5,residual_warning_only=True,bound_joints=True)
            if not d['valid']:failures.append(dict(reason=d['reason'],stage='descent',approach_s=approach,flip=flip));continue
            close_start=int(np.searchsorted(d['times'],descent_s))
            close_errors=[np.linalg.norm(r.ee_fk('right',np.array(row))[:3,3]-(forecast(at_obj,v,t)@grasp)[:3,3]) for t,row in zip(d['times'][close_start:],d['path'][close_start:])]
            # Nominal closure residual is diagnostic; live position gates hand closure.
            close_warning = bool(max(close_errors)>CLOSE_POSITION_TOLERANCE_M)
            ft=np.r_[a['times'],approach+np.array(d['times'])[1:]]
            fp=np.vstack([a['path'],np.array(d['path'])[1:]]);fv=np.vstack([a['velocities'],np.array(d['velocities'])[1:]])
            ft,fp,fv,brake=add_brake(r,ft,fp,fv)
            full=dict(times=ft.tolist(),path=fp.tolist(),velocities=fv.tolist())
            try:margin=validate(c,full,r)
            except ValueError as e:failures.append(dict(reason=str(e),stage='full_path'));continue
            limits=r.limits[14:21];right=fp[:,14:21]
            joint_margin=float(np.min(np.minimum(right-limits[:,0],limits[:,1]-right)))
            if joint_margin<np.deg2rad(3):failures.append(dict(reason='less than 3deg nominal joint reserve',margin_deg=float(np.degrees(joint_margin))));continue
            # Execute only approach + tracking + checked braking. Descent still requires live admission.
            upper=integrate(r,candidate_config,q,obj,v,grasp,approach,10.,tracking_window_s=.8,residual_warning_only=True)
            if not upper['valid']:failures.append(dict(reason=upper['reason'],stage='upper'));continue
            ts,path,vel,brake=add_brake(r,upper['times'],upper['path'],upper['velocities'])
            result=dict(times=ts.tolist(),path=path.tolist(),velocities=vel.tolist(),initial_pose=initial.tolist(),grasp=grasp.tolist(),
                        brake_start_s=float(upper['times'][-1]),execution_end_s=min(10.,approach+duration+1.5),approach_s=approach,tracking_s=.8,braking_s=brake,flip=flip,
                        object_at_start=obj.tolist(),velocity=v.tolist(),geometry=geometry,tracking_lookahead_s=0.,
                        intercept=dict(approach_s=approach,object_position=(forecast(obj,v,approach)[:3,3]).tolist(),nominal_close_position=closure_point.tolist(),
                                       full_path_validated=True,nominal_descent_s=descent_s,min_joint_margin_deg=float(np.degrees(joint_margin)),min_conveyor_margin_m=margin,close_position_error_m=float(max(close_errors)),close_position_warning=close_warning,close_position_policy='warning_only',live_close_tolerance_m=CLOSE_POSITION_TOLERANCE_M,
                                       zone=zone,planning_ms=(time.perf_counter()-began)*1000))
            validate(c,result,r)
            result['cartesian_residual']=dict(policy='warning_only',position_m=upper['max_position_error_m'],rotation_rad=upper['max_rotation_error_rad'])
            return result
    error='no feasible intercept approach+descent+closure: '+json.dumps(failures,ensure_ascii=False)
    raise ValueError(error)


def policy_upper(r,c,q,obj,v,grasp,approach,duration,initial,geometry,flip):
    from async_upper import validate
    from initial_seed_trial import references
    fixed=None;planning_obj=obj;planning_v=v
    if c['tracking_grasp_policy'].get('fixed_interception'):
        from fixed_interception import candidates
        fixed=c.get('_fixed_candidate')
        if fixed is None:
            try:choices=candidates(obj,v,grasp,approach,c['approach_height'],c['tracking_grasp_policy'],initial[:3,3])
            except ValueError as e:return None,[str(e)]
            errors=[]
            for _,candidate in choices:
                result,fail=policy_upper(r,dict(c,_fixed_candidate=candidate),q,obj,v,grasp,approach,duration,initial,geometry,flip)
                errors.extend(fail)
                if result is not None:return result,errors
            return None,errors
        planning_obj=np.asarray(fixed['pose'])@np.linalg.inv(grasp)
        planning_v=np.zeros(4);c=dict(c,_workspace_follow=True,tracking_lookahead_s=0.)
    seeds,compared=references(r,c,q,planning_obj,planning_v,grasp,approach)
    candidates=[];failures=[]
    if not roi_contains(geometry,[forecast(obj,v,t)[:3,3] for t in np.linspace(0,approach+.8,20)]):
        return None,['upper tracking forecast leaves observation ROI']
    for seed in seeds:
        if 'reference' not in seed:continue
        cfg=dict(c,_initial_posture_reference=seed['reference'])
        began=time.perf_counter()
        upper=integrate(r,cfg,q,planning_obj,planning_v,grasp,approach,10.,tracking_window_s=.8,residual_warning_only=True,check_conveyor=False)
        if not upper['valid']:
            failures.append(dict(seed=seed['name'],reason=upper['reason']));continue
        ts,path,vel,brake=add_brake(r,upper['times'],upper['path'],upper['velocities'])
        result=dict(times=ts.tolist(),path=path.tolist(),velocities=vel.tolist(),initial_pose=initial.tolist(),
            grasp=grasp.tolist(),brake_start_s=float(upper['times'][-1]),execution_end_s=10.,approach_s=approach,
            tracking_s=.8,braking_s=brake,flip=flip,object_at_start=obj.tolist(),velocity=v.tolist(),geometry=geometry,
            tracking_lookahead_s=0.,workspace_follow=c['_workspace_follow'])
        if fixed is not None:result['fixed_interception']=fixed
        try:margin=validate(cfg,result,r)
        except ValueError as e:failures.append(dict(seed=seed['name'],reason=str(e)));continue
        arm=path[:,14:21];reserve=float(np.min(np.minimum(arm-r.limits[14:21,0],r.limits[14:21,1]-arm)))
        travel=float(np.sum(np.linalg.norm(np.diff(arm,axis=0),axis=1)))
        result['initial_seed_trial']=dict(compared=compared,seeds=seeds,selected=seed['name'],path_compute_ms=(time.perf_counter()-began)*1000,
                                         min_joint_margin_deg=float(np.degrees(reserve)),joint_travel_rad=travel)
        result['intercept']=dict(approach_s=approach,object_position=forecast(obj,v,approach)[:3,3].tolist(),
            full_path_validated=False,upper_path_validated=True,descent_policy='workspace live replacement',
            nominal_descent_s=duration-.5,min_conveyor_margin_m=margin,zone=c['intercept_window'])
        result['cartesian_residual']=dict(policy='warning_only',position_m=upper['max_position_error_m'],rotation_rad=upper['max_rotation_error_rad'])
        j6_reserve=float(np.min(np.minimum(path[:,19]-r.limits[19,0],r.limits[19,1]-path[:,19])))
        score=20*upper['max_position_error_m']+.1*travel-1.0*reserve-1.0*j6_reserve
        result['initial_seed_trial']['min_j6_margin_deg']=float(np.degrees(j6_reserve))
        candidates.append((score,result))
    if not candidates:return None,failures
    for _,result in sorted(candidates,key=lambda row:row[0]):
        if fixed is not None:
            from fixed_interception import preflight
            try:result['fixed_descent_preflight']=preflight(c,result)
            except ValueError as e:
                failures.append(dict(stage='fixed_descent_preflight',reason=str(e)));continue
        return result,failures
    return None,failures
