"""Compare endpoint IK seeds once per target; never start a path at a demo pose."""
import time
import numpy as np
from deux_task_core.position_first_preview import desired

# A single worker owns this cache; replacement planning never enters this function.
_CACHE={}
_BACKEND=None
_BACKEND_KEY=None

def backend_robot(config):
    global _BACKEND,_BACKEND_KEY
    import json
    key=json.dumps([config['robot'],config.get('ik_backend','pyroki'),config.get('_stationary_left_arm')],sort_keys=True)
    if key!=_BACKEND_KEY:
        from right_only_trial import planning_robot
        from deux_task_core.ik_backends import attach_backend
        robot=planning_robot(config);attach_backend(robot,config)
        _BACKEND=robot;_BACKEND_KEY=key
    return _BACKEND

def warm(config):
    began=time.perf_counter();robot=backend_robot(config);q=np.array(config['home_joints'])
    robot.ik('right',robot.ee_fk('right',q),q)
    return dict(backend=config.get('ik_backend','pyroki'),warmup_s=time.perf_counter()-began)


def references(robot,config,q,obj,v,grasp,approach):
    key=(config.get('_tracking_target_id'),config.get('_seed_config_version'))
    if key in _CACHE:return _CACHE[key],False
    policy=config['tracking_grasp_policy']
    target,_=desired(robot.ee_fk('right',q),obj,v,grasp,config['approach_height'],approach,10.,approach,
                     lookahead=0.,workspace=policy,workspace_follow=config.get('_workspace_follow',False))
    demos=np.asarray(policy['demo_right_arm_rad'],float)
    if demos.ndim!=2 or demos.shape[1]!=7 or not np.isfinite(demos).all():raise ValueError('invalid demo seeds')
    # Pose-nearest demo of the small bank (position + orientation), then one solve.
    from scipy.spatial.transform import Rotation
    ranked=[]
    for arm in demos:
        seed=np.array(q).copy();seed[14:21]=arm;T=robot.ee_fk('right',seed)
        ranked.append((np.linalg.norm(T[:3,3]-target[:3,3])+.1*Rotation.from_matrix(T[:3,:3]@target[:3,:3].T).magnitude(),seed))
    solver=backend_robot(config)
    trial=[('measured',np.array(q).copy()),('demo',min(ranked,key=lambda a:a[0])[1])];result=[]
    for name,seed in trial:
        began=time.perf_counter()
        try:
            solved=solver.ik('right',target,seed)
            arm=solved[14:21];reserve=float(np.min(np.minimum(arm-robot.limits[14:21,0],robot.limits[14:21,1]-arm)))
            if reserve < robot.tracking_joint_reserve_rad:raise ValueError('seed solution lacks joint reserve')
            result.append(dict(name=name,reference=arm.tolist(),ik_ms=(time.perf_counter()-began)*1000,reserve_deg=float(np.degrees(reserve))))
        except ValueError as e:result.append(dict(name=name,error=str(e),ik_ms=(time.perf_counter()-began)*1000))
    # Failed endpoint IK must not disable the existing continuous differential IK.
    if not any('reference' in row for row in result):result.append(dict(name='measured_continuous',reference=q[14:21].tolist()))
    _CACHE.clear();_CACHE[key]=result
    return result,True
