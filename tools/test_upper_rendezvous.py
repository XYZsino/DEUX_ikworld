"""Offline unit and actual-URDF tests. Never construct an action client."""
import json,sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path('tools').resolve()))
from upper_rendezvous import *


@pytest.mark.parametrize("height", [.05,.08])
def test_merge_matches_bag_at_ramp_end_without_extra_lookahead(height):
    fixed=np.eye(4);fixed[:3,3]=[.6,-.1,.92];v=np.array([.002,-.095,0.,0.]);ramp=.4
    for t in [0.,ramp/2,ramp,ramp+.5]:
        p,feed=target(fixed,height,v,t,ramp)
        assert p[2,3]==pytest.approx(.92+height)
        if t>=ramp:
            assert p[:2,3]==pytest.approx(fixed[:2,3]+v[:2]*(t-ramp/2))
            assert feed[:2]==pytest.approx(v[:2])
    for boundary in [0.,ramp]:
        p0,v0=target(fixed,height,v,boundary-1e-7,ramp);p1,v1=target(fixed,height,v,boundary+1e-7,ramp)
        assert np.max(abs(p0-p1))<1e-6;assert np.max(abs(v0-v1))<1e-6


def test_acquisition_time_and_fixed_point_eta():
    fixed=np.eye(4);obj=np.eye(4);obj[1,3]=.2;v=np.array([0.,-.1,0.,0.])
    at,lat=arrival(fixed,obj,v,np.eye(4),100.,100.7)
    assert at==pytest.approx(102.);assert lat==0
    assert arrival(fixed,obj,v,np.eye(4),100.,101.)[0]==pytest.approx(at)
    with pytest.raises(ValueError):arrival(fixed,obj,v,np.eye(4),100.,99.)


def test_fixed_height_retains_raw_evidence_and_cancels_vertical_drift():
    fixed=np.eye(4);fixed[2,3]=.92;obj=fixed.copy();obj[2,3]+=.01;v=np.array([0.,-.1,.01,0.])
    o,w,z=fixed_observation(obj,v,np.eye(4),fixed)
    assert o[2,3]==pytest.approx(.92);assert z==pytest.approx(.93);assert w[2]==0
    assert obj[2,3]==pytest.approx(.93);assert v[2]==.01
    obj[2,3]+=.02
    with pytest.raises(ValueError,match='height'):fixed_observation(obj,v,np.eye(4),fixed)


def test_qualification_requires_relative_speed_and_continuous_fresh_time():
    q=Qualification(DEFAULTS);v=np.array([0.,-.1,0.,0.])
    for t in np.arange(0.,.9,.02):
        p=np.array([.6,-.1*t,.97]);q.update(float(t),p,p,v)
    assert q.passed
    q=Qualification(DEFAULTS)
    for t in np.arange(0.,.9,.02):
        p=np.array([.6,0.,.97]);q.update(float(t),p,p,v)
    assert not q.passed # Position coincidence alone is insufficient.
    q=Qualification(DEFAULTS)
    for t in list(np.arange(0.,.4,.02))+list(np.arange(.6,.95,.02)):
        p=np.array([.6,-.1*t,.97]);q.update(float(t),p,p,v)
    assert not q.passed # Missing feedback cannot count toward the half-second.


def test_old_frames_cannot_accumulate_qualification():
    q=Qualification(DEFAULTS);p=np.zeros(3);v=np.zeros(4)
    for _ in range(100):q.update(1.,p,p,v)
    assert not q.passed


@pytest.fixture(scope='module')
def case():
    from deux_task_core.configuration import load_configuration
    c,_=load_configuration('src/deux_task_bringup/config/installation.yaml')
    template=json.load(open('calibration/reusable_intercept.json'));p=template['plan']
    fixed=np.array(p['fixed_interception']['pose']);nom=np.array(template['nominal_velocity'])
    fixed[:2,3]-=.05*nom[:2]/np.linalg.norm(nom[:2])
    c['approach_height']=json.load(open('calibration/upper_rendezvous.json'))['upper_height_m'];c['tracking_lookahead_s']=0.
    c['tracking_grasp_policy']=json.load(open('calibration/tracking_grasp_policy.json'))
    c['intercept_window']=json.load(open('calibration/intercept_window.json'))
    current=np.array(p['path'][0]);c['_stationary_left_arm']=current[:7].tolist()
    waiting=prepare_wait(c,current,np.array(p['path'][-1]),fixed)
    return c,fixed,waiting,np.array(p['grasp']),p['geometry']


def test_actual_urdf_preposition_without_target(case):
    c,fixed,waiting,g,geometry=case
    r=planning_robot(c);q=np.array(waiting['path'][-1]);desired=fixed[:3,3].copy();desired[2]+=c['approach_height']
    assert r.ee_fk('right',q)[:3,3]==pytest.approx(desired,abs=.005)
    assert np.max(abs(np.array(waiting['velocities'])[-1]))==0
    assert np.max(abs(np.array(waiting['path'])[:,:7]-waiting['path'][0][:7]))<1e-9


@pytest.mark.parametrize('speed',[.03,.06,.095])
def test_actual_urdf_follow_and_brake(case,speed):
    c,fixed,waiting,g,geometry=case;q=np.array(waiting['path'][-1]);v=np.array([0.,-speed,0.,0.])
    p=following_path(c,q,fixed,v,DEFAULTS)
    assert p['max_follow_error_m'][0]<.02
    assert p['max_follow_error_m'][1]<.01
    assert np.max(abs(spline(p)(p['times'][-1],1)))<1e-8
    assert np.max(abs(np.array(p['path'])[:,:7]-q[:7]))<1e-9


def test_dispatch_rejects_deadline_and_speed_changes(case):
    c,fixed,waiting,g,geometry=case;v=np.array([0.,-.06,0.,0.])
    p=following_path(c,np.array(waiting['path'][-1]),fixed,v,DEFAULTS)
    obj=fixed@np.linalg.inv(g);obj[:3,3]-=v[:3]*.2
    with pytest.raises(ValueError,match='late'):dispatch_checks(p,fixed,g,obj,v,100.,100.,geometry,DEFAULTS)
    with pytest.raises(ValueError,match='stale'):dispatch_checks(p,fixed,g,obj,v,100.,101.,geometry,DEFAULTS)
    obj[:3,3]-=v[:3]*2
    changed=v.copy();changed[1]-=.01
    with pytest.raises(ValueError,match='velocity'):dispatch_checks(p,fixed,g,obj,changed,100.,100.,geometry,DEFAULTS)


def test_settings_reject_unsafe_or_nonfinite_values():
    p=dict(DEFAULTS,upstream_offset_m=.05)
    validate_settings(p)
    for key,value in [('xy_m',.03),('ramp_s',float('nan')),('qualify_s',2.),('max_observation_age_s',1.)]:
        with pytest.raises(ValueError):validate_settings(dict(p,**{key:value}))


def test_qualified_tracking_replay_of_validated_joint_path(case):
    c,fixed,waiting,g,geometry=case;r=planning_robot(c);v=np.array([0.,-.095,0.,0.])
    p=following_path(c,np.array(waiting['path'][-1]),fixed,v,DEFAULTS);s=spline(p);qual=Qualification(DEFAULTS)
    for t in np.arange(0,p['times'][-1],.02):
        hand=r.ee_fk('right',s(t))[:3,3]
        bag=fixed[:3,3].copy()+v[:3]*(t-DEFAULTS['ramp_s']/2);bag[2]+=c['approach_height']
        qual.update(float(t),hand,bag,v,eligible=p['merge_s']<=t<=p['brake_start_s'])
    assert qual.passed


def test_settling_uses_position_history_and_rejects_real_motion():
    point=np.array([.6,-.1,.97]);quiet=Settling(point);moving=Settling(point);bad_pose=Settling(point)
    a=b=c=False
    for i,t in enumerate(np.arange(0,1,.025)):
        jitter=np.ones(7)*(.0001 if i%2 else -.0001)
        a,_=quiet.update(float(t),jitter,point+np.array([.008,0,0]))
        b,_=moving.update(float(t),np.ones(7)*.04*t,point)
        c,_=bad_pose.update(float(t),jitter,point+np.array([.011,0,0]))
    assert a and not b and not c


def test_settling_does_not_count_repeated_or_gapped_feedback():
    d=Settling(np.zeros(3));passed=False
    for t in [*np.arange(0,.4,.025),*np.arange(.6,1.,.025)]:
        passed,_=d.update(float(t),np.zeros(7),np.zeros(3))
    assert not passed
    for _ in range(30):assert not d.update(float(t),np.zeros(7),np.zeros(3))[0]


def test_upper_trial_ignores_raw_height_but_uses_fixed_height():
    fixed=np.eye(4);fixed[2,3]=.9184770866
    for z in [.88,.95,1.01]:
        obj=fixed.copy();obj[2,3]=z
        o,v,raw=fixed_observation(obj,np.array([0.,-.1,.02,0.]),np.eye(4),fixed,None)
        assert raw==pytest.approx(z)
        assert o[2,3]==pytest.approx(fixed[2,3])
        assert v[2]==0


def test_upper_trial_ignores_raw_height_but_uses_fixed_height():
    fixed=np.eye(4);fixed[2,3]=.9184770866
    for z in [.88,.95,1.01]:
        obj=fixed.copy();obj[2,3]=z
        o,v,raw=fixed_observation(obj,np.array([0.,-.1,.02,0.]),np.eye(4),fixed,None)
        assert raw==pytest.approx(z)
        assert o[2,3]==pytest.approx(fixed[2,3])
        assert v[2]==0
