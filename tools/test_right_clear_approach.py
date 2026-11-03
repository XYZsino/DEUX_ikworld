"""Mandatory robot-right route, measured-start URDF validation; no robot commands."""
import json,sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path('tools').resolve()))
from right_clear_approach import waypoints,plan_right_approach
from right_only_trial import planning_robot
from async_upper import spline
from deux_task_core.configuration import load_configuration


@pytest.fixture(scope='module')
def route():
    c,_=load_configuration('src/deux_task_bringup/config/installation.yaml')
    snap=json.load(open('.setup/upper-right-route/start.json'));q=np.array(snap['q'])
    c['_stationary_left_arm']=q[:7].tolist();c['intercept_window']=json.load(open('calibration/intercept_window.json'))
    p=json.load(open('calibration/reusable_intercept.json'));target=np.array(p['plan']['fixed_interception']['pose']);v=np.array(p['nominal_velocity'])
    target[:2,3]-=v[:2]/np.linalg.norm(v[:2])*.05;target[2,3]+=.08
    r=planning_robot(c);plan=plan_right_approach(c,q,target)
    return c,r,q,target,plan


def test_right_escape_keeps_xz_before_lifting(route):
    c,r,q,target,p=route;s=p['right_approach']['stages']
    assert [x['name'] for x in s]==['RightEscape','RaiseOutsideConveyor','AdvanceOnRight','EnterUpperWait']
    curve=spline(p);positions=np.array([r.ee_fk('right',curve(t))[:3,3] for t in np.linspace(0,s[0]['end_s'],101)])
    assert positions[-1,1]<positions[0,1]-.15
    assert np.max(abs(positions[:,[0,2]]-positions[0,[0,2]]))<.005
    assert np.max(np.diff(positions[:,1]))<.0001


def test_lift_and_forward_stay_in_right_corridor(route):
    c,r,q,target,p=route;s=p['right_approach']['stages'];y=p['right_approach']['right_corridor_y_m'];curve=spline(p)
    for stage in s[1:3]:
        for t in np.linspace(stage['start_s'],stage['end_s'],80):
            xyz=r.ee_fk('right',curve(t))[:3,3]
            assert xyz[1]<=y+.005
            if stage['name']=='AdvanceOnRight':assert xyz[2]>=target[2,3]-.005
    assert np.linalg.norm(r.ee_fk('right',p['path'][-1])[:3,3]-target[:3,3])<.005
    assert p['min_conveyor_margin_m']>0


def test_route_preserves_left_and_stops_at_every_join(route):
    c,r,q,target,p=route;curve=spline(p)
    assert np.max(abs(np.array(p['path'])[:,:7]-q[:7]))<1e-10
    for stage in p['right_approach']['stages']:assert np.max(abs(curve(stage['end_s'],1)))<1e-8


def test_invalid_clearance_rejected(route):
    c,r,q,target,p=route
    with pytest.raises(ValueError,match='clearance'):waypoints(r.ee_fk('right',q),target,.005)


def test_no_direct_fallback_when_right_segment_blocked(route,monkeypatch):
    import right_clear_approach as module
    c,r,q,target,p=route;attempts=[]
    def blocked(*args):
        attempts.append(args[-1]);raise ValueError('blocked right corridor')
    monkeypatch.setattr(module,'segment',blocked)
    with pytest.raises(ValueError,match='blocked right corridor'):plan_right_approach(c,q,target)
    assert attempts==['RightEscape']
