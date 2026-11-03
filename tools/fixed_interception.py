"""Fixed interception point; pure planning helpers, no robot commands.
All times are relative to the original trajectory epoch.
"""
import numpy as np
from deux_task_core.conveyor import forecast
from deux_task_core.position_first_preview import desired
from deux_task_core.tracking_policy import inside_xy
from scheduled_descent import profile_for
from progressive_descent import height_state


def close_offset(profile, height):
    if profile['height1'] > height:
        raise ValueError('fixed interception: descent cannot reach closing height')
    lo=profile['start'];hi=lo+profile['duration']
    for _ in range(40):
        mid=(lo+hi)/2
        if height_state(profile,mid,profile['height0'])[0]>height:lo=mid
        else:hi=mid
    return hi-profile['start']


def candidates(obj,v,grasp,approach,height,policy,reference=None):
    """Sample spatial belt/workspace intersection, not a fixed future duration."""
    point=(obj@grasp)[:3,3];vel=np.asarray(v[:2],float)
    lo=np.asarray(policy['xy_min_m'])+policy['entry_margin_m']
    hi=np.asarray(policy['xy_max_m'])-policy['entry_margin_m']
    # Reserve room for the following close interval in the travel direction.
    shift=vel*policy['close_duration_s'];lo-=np.minimum(shift,0);hi-=np.maximum(shift,0)
    begin=-float('inf');end=float('inf')
    for k in range(2):
        if abs(vel[k])<1e-8:
            if not lo[k]<=point[k]<=hi[k]:raise ValueError('belt line misses grasp workspace')
        else:
            bounds=sorted([(lo[k]-point[k])/vel[k],(hi[k]-point[k])/vel[k]])
            begin=max(begin,bounds[0]);end=min(end,bounds[1])
    if not np.isfinite([begin,end]).all() or begin>end:raise ValueError('no moving belt intersection')
    profile=profile_for(height,0.,policy['descent_speed_m_s'],policy['max_descent_m'])
    needed=approach+.5+close_offset(profile,policy['close_height_m'])
    out=[]
    # Interior spatial fractions stay invariant when the observation moves along the same line.
    for fraction in (.25,.5,.75):
        contact=begin+(end-begin)*fraction
        if contact<needed:continue
        frozen=forecast(obj,v,contact);pose=frozen@grasp
        score=float(np.linalg.norm(pose[:2,3]-np.asarray(reference)[:2])) if reference is not None else abs(fraction-.5)
        out.append((score,frozen,dict(pose=pose.tolist(),nominal_contact_s=float(contact),workspace_fraction=fraction)))
    if not out:raise ValueError('no workspace candidate reachable before bag arrival')
    return [(o,f) for _,o,f in sorted(out,key=lambda item:item[0])]


def select(obj,v,grasp,approach,height,policy):
    return candidates(obj,v,grasp,approach,height,policy)[0]


def refresh(fixed,obj,v,grasp,approach,height,policy):
    """Keep point immutable; update only timing at a new trajectory epoch."""
    contact,lateral=arrival(fixed,obj,v,grasp,0.)
    required=approach+.5+close_offset(profile_for(height,0.,policy['descent_speed_m_s'],policy['max_descent_m']),policy['close_height_m'])
    if lateral>policy['close_xy_m'] or contact<required:
        raise ValueError('selected point no longer reachable before arrival')
    pose=np.asarray(fixed['pose'])
    if not inside_xy(policy,[pose[:3,3],pose[:3,3]+np.asarray(v[:3])*policy['close_duration_s']],policy['entry_margin_m']):
        raise ValueError('selected point closure leaves workspace')
    return dict(fixed,nominal_contact_s=float(contact),required_ready_s=float(required),time_slack_s=float(contact-required))


def rebase(plan,obj,v,stamp,start,policy,height):
    """Retiming is valid only for stationary upper target, before any descent."""
    if not plan.get('fixed_interception') or plan.get('descent_profile') is not None:
        raise ValueError('only fixed upper plan can be rebased')
    at_start=forecast(np.asarray(obj),np.asarray(v),start-stamp)
    fixed=refresh(plan['fixed_interception'],at_start,np.asarray(v),np.asarray(plan['grasp']),plan['approach_s'],height,policy)
    return dict(plan,fixed_interception=fixed,object_at_start=at_start.tolist(),velocity=np.asarray(v).tolist(),source_object=np.asarray(obj).tolist(),source_stamp=float(stamp),start_at=float(start))


def arrival(fixed,obj,v,grasp,t):
    point=(forecast(obj,v,t)@grasp)[:3,3]
    delta=np.asarray(fixed['pose'])[:3,3]-point
    velocity=np.asarray(v[:2]);speed2=float(velocity@velocity)
    if speed2<.005**2:
        if np.linalg.norm(delta[:2])>.02:raise ValueError('fixed interception: stationary bag away from point')
        return t,0.
    eta=float(delta[:2]@velocity/speed2)
    lateral=float(np.linalg.norm(delta[:2]-velocity*eta))
    return t+eta,lateral


def schedule(fixed,obj,v,grasp,splice,height,policy):
    contact,lateral=arrival(fixed,obj,v,grasp,splice)
    profile=profile_for(height,0.,policy['descent_speed_m_s'],policy['max_descent_m'])
    start=contact-close_offset(profile,policy['close_height_m'])
    if lateral>policy['close_xy_m']:raise ValueError('fixed interception: belt misses selected point')
    if start<splice:raise ValueError('fixed interception: too late for scheduled descent')
    profile['start']=start
    profile['contact_s']=contact
    profile['follow_velocity']=np.asarray(v).tolist()
    return profile


def target(initial,fixed,height,approach,t,profile=None,hold_rotation=None):
    pose,feed=desired(initial,np.asarray(fixed['pose']),np.zeros(4),np.eye(4),height,approach,10.,t,lookahead=0.)
    if profile is not None and t>=profile['start']:
        h,hd=height_state(profile,t,height)
        pose[2,3]=np.asarray(fixed['pose'])[2,3]+h;feed[2]=hd
        # Remain over the chosen point until contact, then smoothly match belt speed.
        u=max(0.,t-profile['contact_s']);r=.2
        distance=u*u/(2*r) if u<r else u-r/2
        rate=min(1.,u/r)
        v=np.asarray(profile['follow_velocity'])
        pose[:2,3]+=v[:2]*distance;feed[:2]=v[:2]*rate
    if hold_rotation is not None:pose[:3,:3]=hold_rotation;feed[3:]=0.
    return pose,feed

def preflight(config,plan):
    """Check nominal descent/follow before selecting a point; never dispatch."""
    from async_upper import compute_replacement
    c=dict(config,_descent_admitted=True,_descent_measured_height=config['approach_height'])
    candidate=dict(plan,start_at=0.,target_id='preflight',generation=0)
    splice=plan['approach_s']+.3
    for generation in range(1,16):
        candidate=compute_replacement(c,candidate,np.asarray(plan['object_at_start']),np.asarray(plan['velocity']),0.,splice,generation)
        profile=candidate.get('descent_profile')
        if profile is None:
            splice=candidate['brake_start_s']-.25
            continue
        finish=max(profile['start']+profile['duration'],profile['contact_s']+config['tracking_grasp_policy']['close_duration_s'])
        if candidate['brake_start_s']>=finish:
            from right_only_trial import planning_robot
            from async_upper import spline
            robot=planning_robot(c);curve=spline(candidate)
            errors=[]
            for t in np.linspace(profile['contact_s'],finish,8):
                hand=robot.ee_fk('right',curve(t))
                bag=forecast(np.asarray(plan['object_at_start']),np.asarray(plan['velocity']),t)@np.asarray(plan['grasp'])
                errors.append(float(np.linalg.norm(hand[:2,3]-bag[:2,3])))
            if max(errors)>config['tracking_grasp_policy']['close_xy_m']:
                raise ValueError('nominal fixed-point closure XY residual exceeds tolerance')
            return dict(valid=True,contact_s=profile['contact_s'],finish_s=finish,close_xy_max_m=max(errors))
        splice=candidate['brake_start_s']-.25
    raise ValueError('nominal descent preview exhausted')
