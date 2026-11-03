import json
from types import SimpleNamespace as S
import numpy as np
from deux_task_runtime.tracking_trace import TrackingTrace
from deux_task_core.model import ARM_NAMES

def test_trace_retains_failure_sample_and_records_feedback_velocity(tmp_path):
    q=np.zeros((2,28));q[1,16]=.2
    tr=TrackingTrace(S(plan_id='test',stage='Approach',scene_version='v'),[0,1],q,10,.15,capacity=2)
    state=S(header=S(stamp=S(sec=10,nanosec=200000000)),name=list(ARM_NAMES),velocity=[.03]*14)
    measured=np.zeros(28);measured[16]=.3
    for now in [10.1,10.2,10.3]:tr.append(now,measured,q[0],state)
    p=tr.save(tmp_path,'tracking error');d=json.loads(p.read_text())
    assert d['samples_dropped']==1 and d['outcome']=='tracking error'
    row=d['samples'][-1];assert row['error_rad'][9]==.3
    assert row['planned_velocity_rad_s'][9]==.2
    assert row['measured_velocity_rad_s'][9]==.03
    assert abs(row['feedback_age_s']-.1)<1e-10
    assert list(tmp_path.glob('*-j3.csv'))

def test_missing_velocity_is_not_reported_as_zero(tmp_path):
    tr=TrackingTrace(S(plan_id='test',stage='Approach',scene_version='v'),[0,1],np.zeros((2,28)),10,.15)
    tr.append(9.9,np.zeros(28),np.zeros(28),None)
    d=json.loads(tr.save(tmp_path,'canceled').read_text());assert d['samples'][0]['measured_velocity_rad_s']==[None]*14
