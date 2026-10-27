"""Observed-velocity conveyor interception; all coordinates are robot-base metres."""
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.interpolate import CubicHermiteSpline
from .model import JOINT_NAMES,HAND_NAMES,pose,vector


def inside(point,bounds,margin=0.):
    p=np.asarray(point,float)
    return p.shape==(3,) and np.isfinite(p).all() and bool(np.all(p>=np.asarray(bounds['min'])+margin) and np.all(p<=np.asarray(bounds['max'])-margin))


def uses_workspace_box(config):
    """Legacy installations may opt into a box; IK mode has no XYZ gate."""
    policy=config['conveyor_tracking'].get('workspace_policy','box')
    if policy not in ('box','ik'):raise ValueError('invalid conveyor workspace_policy')
    return policy=='box'


def forecast(matrix,velocity,dt):
    if not np.isfinite(dt) or dt<0:raise ValueError('invalid forecast interval')
    out=pose(matrix).copy();v=vector(velocity,4)
    out[:3,3]+=v[:3]*dt
    out[:3,:3]=Rotation.from_rotvec([0,0,v[3]*dt]).as_matrix()@out[:3,:3]
    return out


def grasp_entry_prediction(config, object_pose, velocity, observation_age):
    """Pre-plan entry gate using the remaining scheduled motion phases.

    Actual dynamics retiming is checked again by plan_conveyor/plan_track_grasp.
    Both action launches need start_lead. The rendezvous allowance is separate
    from Track; closure overlap changes its start, not the trajectory end.
    """
    cfg=config['conveyor_tracking'];d=cfg['durations']
    if not np.isfinite(observation_age) or observation_age<0:
        raise ValueError('invalid conveyor observation age')
    approach=cfg['start_lead']+d['Approach']
    remaining=approach+cfg.get('intercept_wait_s',0.)+cfg['start_lead']+sum(d[s] for s in ('Track','Descend','Close'))
    arrival_s=observation_age+approach
    completion_s=observation_age+remaining
    arrival=forecast(object_pose,velocity,arrival_s)
    completion=forecast(object_pose,velocity,completion_s)
    failure=None
    if uses_workspace_box(config):
        if not inside(np.asarray(object_pose)[:3,3],cfg['acquisition_region']):failure='acquisition'
        elif not inside(arrival[:3,3],cfg['grasp_region'],cfg['boundary_margin']):failure='arrival'
        elif not inside(completion[:3,3],cfg['grasp_region'],cfg['boundary_margin']):failure='completion'
    return dict(remaining_s=remaining,arrival_s=arrival_s,completion_s=completion_s,
                arrival=arrival,completion=completion,failure=failure)


def conveyor_velocity(measured, config):
    """Use the known belt command only after agreement with measured motion."""
    v=vector(measured,4);cfg=config['conveyor_tracking']
    if cfg.get('occluded_grasp_enabled',False):
        nominal=vector(cfg['nominal_velocity'],4)
        if np.linalg.norm(v[:3]-nominal[:3])>cfg['belt_velocity_tolerance_m_s'] or abs(v[3]-nominal[3])>cfg['belt_yaw_rate_tolerance_rad_s']:
            raise ValueError('observed motion disagrees with conveyor command')
        return nominal
    return v


def rendezvous_ready(object_pose, velocity, age, measured_ee, config):
    """Gate descent by predicted XY at launch (or after an optional Track)."""
    cfg=config['conveyor_tracking']
    dt=age+cfg['start_lead']+cfg['durations']['Track']
    predicted=forecast(object_pose,velocity,dt)
    if uses_workspace_box(config) and not inside(predicted[:3,3],cfg['grasp_region'],cfg['boundary_margin']):return False
    grasp=predicted@pose(config['grasp_object_from_right_ee'])
    ee=pose(measured_ee)
    return (np.linalg.norm(grasp[:2,3]-ee[:2,3])<=cfg['intercept_trigger_distance_m']
            and abs(ee[2,3]-grasp[2,3]-config['approach_height'])<=cfg['intercept_height_tolerance_m'])


def plan_conveyor(planner,stage,start,object_pose,velocity,lead):
    if stage not in ('Approach','Descend','Close'):raise ValueError('invalid conveyor stage')
    c=planner.config;cfg=c['conveyor_tracking'];robot=planner.robot
    q=vector(start,28);v=vector(velocity,4);obj=pose(object_pose)
    if np.linalg.norm(v[:3])>cfg['max_speed']:raise ValueError('conveyor speed outside configured bound')
    if uses_workspace_box(c) and not inside(obj[:3,3],cfg['acquisition_region'] if stage=='Approach' else cfg['grasp_region']):raise ValueError('target outside permitted acquisition/grasp region')
    first=robot.fk(q)[robot.ee['right']]
    hands=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    duration=cfg['durations'][stage]
    for attempt in range(5):
        horizon=lead+duration+(cfg.get('intercept_wait_s',0.) if stage=='Approach' else 0.)
        if horizon>cfg['max_plan_horizon']:raise ValueError('conveyor planning horizon exceeded')
        end=forecast(obj,v,horizon)
        if uses_workspace_box(c) and not inside(end[:3,3],cfg['grasp_region'],cfg['boundary_margin']):raise ValueError('target exits right grasp region during grasp attempt')
        target=end@pose(c['grasp_object_from_right_ee'])
        if stage=='Approach':
            target[2,3]+=c['approach_height']
            target[:3,3]+=vector(cfg.get('approach_offset_base_m',[0.,0.,0.]),3)
        rotations=Slerp([0,1],Rotation.from_matrix([first[:3,:3],target[:3,:3]]))
        knots=[q.copy()];seed=q.copy();kt=np.linspace(0,duration,max(3,int(np.ceil(duration/.15))+1))
        hand_goal=np.asarray(c['hand_closed'] if stage=='Close' else c.get('hand_pregrasp',c['hand_open']))
        try:approach_goal=robot.ik('right',target,q,continuity_weight=.005) if stage=='Approach' else None
        except ValueError as error:raise ValueError(f'conveyor intercept IK: target={target.tolist()}, object={obj.tolist()}, velocity={v.tolist()}, horizon={horizon}') from error
        for t in kt[1:]:
            u=t/duration;b=10*u**3-15*u**4+6*u**5
            current=forecast(obj,v,lead+t)@pose(c['grasp_object_from_right_ee'])
            if stage=='Approach':current[2,3]+=c['approach_height']
            # Close follows belt motion while the fingers close; other stages
            # smoothly approach the moving interception pose from measured EE.
            desired=np.eye(4);desired[:3,3]=(1-b)*first[:3,3]+b*current[:3,3]
            desired[:3,:3]=rotations(b).as_matrix()
            # The unloaded approach may rotate the wrist through poses outside
            # the Cartesian IK manifold. Joint interpolation reaches the future
            # intercept; descent/closure then follow the moving Cartesian pose.
            if approach_goal is not None:seed=(1-b)*q+b*approach_goal
            else:seed=robot.ik('right',desired,seed)
            seed[hands]=(1-b)*q[hands]+b*hand_goal
            knots.append(seed.copy())
        knots=np.asarray(knots);slope=np.gradient(knots,kt,axis=0);slope[0]=slope[-1]=0.
        spline=CubicHermiteSpline(kt,knots,slope)
        ts=np.linspace(0,duration,int(np.ceil(duration/c['sample_period']))+1);path=spline(ts)
        vv=np.diff(path,axis=0)/np.diff(ts)[:,None]
        aa=np.diff(np.vstack([np.zeros(28),vv,np.zeros(28)]),axis=0)/np.r_[np.diff(ts),np.diff(ts)[-1]][:,None]
        if np.any(abs(vv)>robot.velocity*.98) or np.any(abs(aa)>robot.acceleration*.98):
            duration*=1.3;continue
        clearance=min(robot.clearance(row,c['obstacles'],c['collision_margin'],0.) for row in path)
        return dict(times=ts,path=path,minimum_clearance=clearance,ik_calls=robot.ik_calls)
    raise ValueError('moving target trajectory exceeds joint dynamics')


def tracking_joint_velocity(robot,q,linear,angular):
    """Local differential kinematics, independent of IK solver convergence error."""
    indices=np.arange(14,21);eps=1e-5
    fk=lambda state:robot.ee_fk('right',state) if hasattr(robot,'ee_fk') else robot.fk(state)[robot.ee['right']]
    base=fk(q);jacobian=np.zeros((6,7))
    for column,index in enumerate(indices):
        shifted=q.copy();shifted[index]+=eps;pose=fk(shifted)
        jacobian[:3,column]=(pose[:3,3]-base[:3,3])/eps
        jacobian[3:,column]=Rotation.from_matrix(pose[:3,:3]@base[:3,:3].T).as_rotvec()/eps
    result=np.zeros(28)
    result[indices]=np.linalg.lstsq(jacobian,np.r_[linear,angular],rcond=1e-4)[0]
    if np.max(abs(jacobian@result[indices]-np.r_[linear,angular]))>.01:
        raise ValueError('cannot match conveyor velocity at interception')
    return result


def plan_track_grasp(planner,start,object_pose,velocity,lead):
    """Refresh after interception, match target speed, descend and close trajectory, without inter-stage holds."""
    c=planner.config;cfg=c['conveyor_tracking'];r=planner.robot
    q=vector(start,28);obj=pose(object_pose);v=vector(velocity,4)
    if np.linalg.norm(v[:3])>cfg['max_speed']:raise ValueError('conveyor speed outside configured bound')
    if uses_workspace_box(c) and not inside(obj[:3,3],cfg['grasp_region']):raise ValueError('target outside right grasp region')
    hands=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    durations=np.array([cfg['durations'][s] for s in ('Track','Descend','Close')])
    pregrasp=np.asarray(c.get('hand_pregrasp',c['hand_open']))
    transform=pose(c['grasp_object_from_right_ee'])
    for attempt in range(6):
        ends=np.cumsum(durations);total=ends[-1]
        if lead+total>cfg['max_plan_horizon']:raise ValueError('conveyor planning horizon exceeded')
        if uses_workspace_box(c) and not inside(forecast(obj,v,lead+total)[:3,3],cfg['grasp_region'],cfg['boundary_margin']):
            raise ValueError(f'target exits grasp region before fingers close: phases={durations.tolist()}, lead={lead}, velocity={v.tolist()}')
        approach=forecast(obj,v,lead+ends[0])@transform;approach[2,3]+=c['approach_height']
        goal=r.ik('right',approach,q,continuity_weight=.02) if durations[0]>0 else q.copy()
        goal[hands]=pregrasp
        predicted_object=forecast(obj,v,lead+ends[0])
        angular=np.array([0.,0.,v[3]])
        linear=v[:3]+np.cross(angular,approach[:3,3]-predicted_object[:3,3])
        end_velocity=tracking_joint_velocity(r,goal,linear,angular) if durations[0]>0 else np.zeros(28)
        initial=r.fk(q)[r.ee['right']]
        initial_target=forecast(obj,v,lead)@transform
        initial_rotation=Rotation.from_matrix(initial[:3,:3]@initial_target[:3,:3].T).as_rotvec()
        kt=np.unique(np.r_[0.,ends[1]-durations[1]*cfg.get('close_descent_overlap',0.),*[np.linspace(a,b,max(3,int(np.ceil((b-a)/.12))+1))[1:] for a,b in zip(np.r_[0.,ends[:-1]],ends)]])
        knots=[q.copy()];seed=q.copy()
        for t in kt[1:]:
            if t<=ends[0]:
                u=t/ends[0];b=10*u**3-15*u**4+6*u**5
                seed=(1-b)*q+b*goal+ends[0]*end_velocity*(-4*u**3+7*u**4-3*u**5)
            else:
                # Smoothly stop the arm during the final part of finger closure.
                # The bounded lag must fit the same measured target-drift guard;
                # it is not treated as evidence that contact/grasp succeeded.
                stop=min(.5,durations[2]);lag=0.
                if t>total-stop:
                    u=(t-(total-stop))/stop;lag=stop*(2.5*u**4-3*u**5+u**6)
                if np.linalg.norm(v[:3])*stop/2>c['max_target_drift']*.95:
                    raise ValueError('closing deceleration exceeds tracking error bound')
                desired=forecast(obj,v,lead+t-lag)@transform
                if t<=ends[1]:
                    u=(t-ends[0])/durations[1];b=10*u**3-15*u**4+6*u**5
                    if durations[0]>0:
                        desired[2,3]+=c['approach_height']*(1-b)
                    else:
                        # Start descent from measured pose, matching belt speed
                        # smoothly during descent rather than a separate Track.
                        desired[:3,3]+=(1-b)*(initial[:3,3]-initial_target[:3,3]-v[:3]*t)
                        desired[:3,:3]=Rotation.from_rotvec((1-b)*initial_rotation).as_matrix()@desired[:3,:3]
                # Start closure only in the last part of descent, with no hold
                # between descending and gripping. Retiming still checks fingers.
                close_start=ends[1]-durations[1]*cfg.get('close_descent_overlap',0.)
                u=np.clip((t-close_start)/(total-close_start),0.,1.)
                b=10*u**3-15*u**4+6*u**5
                hand=(1-b)*pregrasp+b*np.asarray(c['hand_closed'])
                seed=r.ik('right',desired,seed,continuity_weight=.02);seed[hands]=hand
            knots.append(seed.copy())
        knots=np.asarray(knots);slopes=np.gradient(knots,kt,axis=0);slopes[0]=slopes[-1]=0.
        close_start=ends[1]-durations[1]*cfg.get('close_descent_overlap',0.)
        slopes[np.ix_(np.flatnonzero(kt<=close_start+1e-9),hands)]=0.
        spline=CubicHermiteSpline(kt,knots,slopes)
        ts=np.linspace(0,total,int(np.ceil(total/c['sample_period']))+1);path=spline(ts)
        vv=np.diff(path,axis=0)/np.diff(ts)[:,None]
        aa=np.diff(np.vstack([np.zeros(28),vv,np.zeros(28)]),axis=0)/np.r_[np.diff(ts),np.diff(ts)[-1]][:,None]
        factor=max(np.max(abs(vv)/r.velocity),np.sqrt(np.max(abs(aa)/r.acceleration)))
        planner.tracking_diagnostics=dict(duration=float(total),factor=float(factor),max_acceleration=float(np.max(abs(aa))),acceleration_index=list(map(int,np.unravel_index(np.argmax(abs(aa)/r.acceleration),aa.shape))))
        if factor> .98:
            # Retiming only the constrained phase avoids extending the whole
            # prediction horizon because one descent or finger joint is slower.
            for index,(begin,end) in enumerate(zip(np.r_[0.,ends[:-1]],ends)):
                if end<=begin:continue
                accel_mask=(ts>=begin-1e-9)&(ts<=end+1e-9)
                velocity_mask=(ts[:-1]>=begin-1e-9)&(ts[:-1]<=end+1e-9)
                local=max(np.max(abs(vv[velocity_mask])/r.velocity),np.sqrt(np.max(abs(aa[accel_mask])/r.acceleration)))
                if local>.98:durations[index]*=max(1.03,float(local)*1.05)
            continue
        clearance=min(r.clearance(row,c['obstacles'],c['collision_margin'],0.) for row in path)
        return dict(times=ts,path=path,phase_end_times=ends,minimum_clearance=clearance,ik_calls=r.ik_calls)
    raise ValueError('continuous conveyor path exceeds dynamics')


def validate_occlusion_prediction(obj,plan,age,now,config):
    """Bound prediction without relabeling an old acquisition as fresh."""
    from scipy.spatial.transform import Rotation
    cfg=config['conveyor_tracking']
    if not 0<=age<=cfg['occlusion_prediction_max_age'] or not obj.motion_covariance_valid:
        raise ValueError('unbounded or uncertified motion prediction')
    covariance=np.asarray(obj.motion_covariance).reshape(8,8)
    if not np.isfinite(covariance).all() or not np.allclose(covariance,covariance.T) or np.linalg.eigvalsh(covariance).min()< -1e-9:
        raise ValueError('invalid motion covariance')
    belt=cfg.get('occluded_grasp_enabled',False)
    f=np.eye(8);f[:4,4:]=np.eye(4)*age
    if belt:
        # Belt command is an independent velocity prior. Preserve measured pose
        # uncertainty; do not propagate the less certain image velocity instead.
        projected=np.zeros((8,8));projected[:4,:4]=covariance[:4,:4]
        projected[:3,:3]+=np.eye(3)*(cfg['belt_velocity_std_m_s']*age)**2
        projected[3,3]+=(cfg['belt_yaw_rate_std_rad_s']*age)**2
    else:projected=f@covariance@f.T
    projected[:3,:3]+=np.eye(3)*(.5*cfg['prediction_acceleration_std']*age**2)**2
    projected[3,3]+=(.5*cfg.get('prediction_yaw_acceleration_std',.15)*age**2)**2
    radius=2*np.sqrt(max(0.,np.linalg.eigvalsh(projected[:3,:3]).max()))
    def matrix(pose):
        p=pose.position;q=pose.orientation;out=np.eye(4)
        out[:3,3]=[p.x,p.y,p.z];out[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();return out
    def velocity(v):return [v.linear.x,v.linear.y,v.linear.z,v.angular.z]
    if belt:
        nominal=vector(cfg['nominal_velocity'],4)
        if np.linalg.norm(np.asarray(velocity(obj.velocity))[:3]-nominal[:3])>cfg['belt_velocity_tolerance_m_s']:
            raise ValueError('last observed motion disagrees with belt speed')
        if not np.allclose(velocity(plan.target_velocity),nominal):raise ValueError('plan does not use belt prediction')
    predicted=forecast(matrix(obj.pose.pose),cfg['nominal_velocity'] if belt else velocity(obj.velocity),age)
    acquired=plan.target_acquired_at.sec+plan.target_acquired_at.nanosec*1e-9
    expected=forecast(matrix(plan.target_pose),velocity(plan.target_velocity),now-acquired)
    if np.linalg.norm(predicted[:3,3]-expected[:3,3])+radius>config['max_target_drift']:
        raise ValueError('occlusion forecast uncertainty exceeds tracking margin')
    angle=Rotation.from_matrix(predicted[:3,:3]@expected[:3,:3].T).magnitude()
    if angle+2*np.sqrt(max(0.,projected[3,3]))>config.get('max_target_angle_rad',.15):raise ValueError('occlusion orientation uncertainty exceeds margin')
    if uses_workspace_box(config) and not inside(predicted[:3,3],cfg['acquisition_region'] if plan.stage=='Approach' else cfg['grasp_region'],cfg['boundary_margin']+radius):
        raise ValueError('occlusion forecast leaves grasp region')

    return float(radius)
