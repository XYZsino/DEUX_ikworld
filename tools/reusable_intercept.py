"""Read-only admission of a prevalidated fixed upper path; no online search."""
import copy,hashlib,json,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from deux_task_core.conveyor import forecast
from deux_task_core.live_continuous_preview import roi_contains
from fixed_interception import refresh


def signature(config):
    keys=('robot','obstacles','collision_margin','approach_height','intercept_window','home_joints')
    data={k:config.get(k) for k in keys}
    data['tracking_grasp_policy']={k:v for k,v in config['tracking_grasp_policy'].items() if k!='reusable_intercept_path'}
    return hashlib.sha256(json.dumps(data,sort_keys=True).encode()).hexdigest()


def admit(config,template,q,obj,v,advance,geometry):
    began=time.perf_counter()
    if template['signature']!=signature(config):raise ValueError('reusable path configuration changed; offline rebuild required')
    q=np.asarray(q);obj=forecast(np.asarray(obj),np.asarray(v),advance);v=np.asarray(v)
    out=copy.deepcopy(template['plan']);start=np.asarray(out['path'][0]);grasp=np.asarray(out['grasp']);fixed=out['fixed_interception']
    if np.max(abs(q[14:21]-start[14:21]))>.01:raise ValueError('reusable path requires ready pose within 0.57deg')
    if np.max(abs(q[:7]-start[:7]))>.02:raise ValueError('reusable collision geometry changed; offline rebuild required')
    wanted=obj@grasp;chosen=np.asarray(fixed['pose'])
    angle=Rotation.from_matrix(wanted[:3,:3]@chosen[:3,:3].T).magnitude()
    if angle>np.deg2rad(15):raise ValueError('bag direction outside reusable orientation band (+/-15deg)')
    if abs(wanted[2,3]-chosen[2,3])>.015:raise ValueError('bag height differs from fixed grasp height by >1.5cm')
    if np.linalg.norm(v[:3]-np.asarray(template['nominal_velocity'])[:3])>.03:raise ValueError('belt speed outside reusable preview band')
    p=config['tracking_grasp_policy'];fixed=refresh(fixed,obj,v,grasp,out['approach_s'],config['approach_height'],p)
    end=fixed['nominal_contact_s']+p['close_duration_s']+.6
    if not roi_contains(geometry,[forecast(obj,v,t)[:3,3] for t in np.linspace(0,end,16)]):raise ValueError('bag leaves ROI before fixed interception completes')
    out.update(fixed_interception=fixed,object_at_start=obj.tolist(),velocity=v.tolist(),geometry=geometry,
               reused_path=True,reuse_admission_ms=(time.perf_counter()-began)*1000)
    return out


_PREPARED=None

def load_and_admit(config,q,obj,v,advance,geometry):
    path=config['tracking_grasp_policy']['reusable_intercept_path']
    record=_PREPARED if _PREPARED is not None and _PREPARED['signature']==signature(config) else json.loads(Path(path).read_text())
    return admit(config,record,q,obj,v,advance,geometry)


def prepare(config,q):
    """One checked start correction after ready, before accepting any bag."""
    global _PREPARED
    record=json.loads(Path(config['tracking_grasp_policy']['reusable_intercept_path']).read_text())
    if record['signature']!=signature(config):raise ValueError('reusable configuration changed; rebuild required')
    from preplanned_entry import correct
    from fixed_interception import preflight
    began=time.perf_counter();p=record['plan'];q=np.asarray(q)
    if np.max(abs(q[:7]-np.asarray(p['path'][0])[:7]))>.02:
        raise ValueError('reusable collision geometry changed; offline rebuild required')
    if np.max(abs(q[14:21]-np.asarray(p['path'][0])[14:21]))>.005:
        p=correct(config,p,q,np.asarray(p['object_at_start']),np.asarray(record['nominal_velocity']),0.,p['geometry'])
        p['fixed_descent_preflight']=preflight(config,p)
        record=dict(record,plan=p)
    _PREPARED=record
    return dict(prepared=True,seconds=time.perf_counter()-began,fixed_pose=p['fixed_interception']['pose'])
