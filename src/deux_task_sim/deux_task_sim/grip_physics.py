"""Explicit simulation-only hand material/servo experiments; no hardware calibration."""
import numpy as np


def apply_grip_physics(sim, settings=None):
    settings=settings or {}
    limit=float(settings.get('right_hand_torque_limit_nm',5.))
    gain=float(settings.get('right_hand_stiffness_scale',1.))
    friction=float(settings.get('finger_sliding_friction_scale',1.))
    if not np.isfinite([limit,gain,friction]).all() or not (0<limit<=20 and 0<gain<=4 and 0<friction<=3):
        raise ValueError('invalid simulation grip parameters')
    model=sim.model;ids=sim._hand_u['right']
    geoms=[i for i in range(model.ngeom) if any(model.body(int(model.geom_bodyid[i])).name.startswith('right_hand_'+f) for f in ('thumb','index','third'))]
    if not geoms:raise ValueError('right finger collision geometry missing')
    # Snapshot once so repeated installation never compounds material or gains.
    if not hasattr(sim,'_grip_physics_baseline'):
        sim._grip_physics_baseline=(model.actuator_gainprm[ids].copy(),model.actuator_biasprm[ids].copy(),model.geom_friction[geoms].copy())
    gains,biases,materials=sim._grip_physics_baseline
    model.actuator_gainprm[ids]=gains;model.actuator_biasprm[ids]=biases
    model.actuator_gainprm[ids,0]=gains[:,0]*gain
    model.actuator_biasprm[ids,1]=biases[:,1]*gain
    model.actuator_biasprm[ids,2]=biases[:,2]*np.sqrt(gain)
    model.actuator_forcerange[ids]=[-limit,limit]
    model.geom_friction[geoms]=materials
    model.geom_friction[geoms,0]=materials[:,0]*friction
    return dict(simulation_only=True,right_hand_torque_limit_nm=limit,
                right_hand_stiffness_scale=gain,finger_sliding_friction_scale=friction,
                original_kp=gains[:,0].tolist(),applied_kp=model.actuator_gainprm[ids,0].tolist(),
                original_sliding_friction=sorted(set(materials[:,0].tolist())),
                applied_sliding_friction=sorted(set(model.geom_friction[geoms,0].tolist())))
