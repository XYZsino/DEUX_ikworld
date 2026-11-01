"""Offline timing, identity and real-URDF path regressions; no action clients."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path('tools').resolve()))
import upper_x_alignment as m
from test_upper_rendezvous import case
from upper_rendezvous import DEFAULTS
from async_upper import spline
from right_only_trial import planning_robot


def test_crossing_uses_acquisition_stamp_and_belt_slope():
    f=np.eye(4);o=np.eye(4);o[:3,3]=[.6,.2,0];v=np.array([.01,-.1,0,0])
    x,at=m.crossing(f,o,v,np.eye(4),100,100.2)
    assert x==pytest.approx(.62) and at==pytest.approx(102)
    for now in [99,100.6]:
        with pytest.raises(ValueError):m.crossing(f,o,v,np.eye(4),100,now)


def test_window_identity_duplicates_jitter_and_gap():
    w=m.CrossingWindow()
    assert w.update('a',1,.6) is None
    assert w.update('a',1,.6) is None
    assert w.update('a',1.1,.601) is None
    assert w.update('a',1.2,.602)==pytest.approx(.601)
    assert w.update('b',1.3,.602) is None
    assert w.update('b',1.4,.62) is None
    assert w.update('b',1.5,.602) is None
    assert w.update('b',2,.602) is None


def test_deadline_rejects_before_motion_and_roi_uses_actual_brake(monkeypatch):
    f=np.eye(4);o=np.eye(4);o[1,3]=.5;v=np.array([0,-.1,0,0]);points=[]
    monkeypatch.setattr(m,'roi_contains',lambda g,p:points.extend(p) or True)
    p={'times':[0,1],'x_alignment':{'following_duration_s':3}}
    at,needed=m.admission(p,f,np.eye(4),o,v,100,100,{},DEFAULTS)
    assert at==105 and needed>2.5
    assert points[-1][1]==pytest.approx(-.28)
    o[1,3]=.2
    with pytest.raises(m.AlignmentTooLate):m.admission(p,f,np.eye(4),o,v,100,100,{},DEFAULTS)


def test_out_of_bounds_not_clamped(case):
    c,f,*_=case
    with pytest.raises(ValueError):m.select_point(f,f[0,3]+.061,c)
    with pytest.raises(ValueError,match='작업영역'):m.select_point(f,f[0,3]+.02,c)


@pytest.mark.parametrize('dx',[-.02,-.03])
def test_real_urdf_x_alignment_preserves_height_and_validates_follow(case,dx):
    c,f,wait,g,geometry=case;r=planning_robot(c);q=np.array(wait['path'][-1])
    selected=m.select_point(f,f[0,3]+dx,c)
    p=m.build_alignment(c,q,selected,np.array([0,-.06,0,0]),DEFAULTS)
    curve=spline(p);xyz=np.array([r.ee_fk('right',curve(t))[:3,3] for t in np.linspace(0,p['times'][-1],101)])
    assert np.max(abs(xyz[:,1]-f[1,3]))<.005
    assert np.max(abs(xyz[:,2]-f[2,3]-c['approach_height']))<.005
    assert xyz[-1,0]==pytest.approx(selected[0,3],abs=.005)
    assert p['x_alignment']['following_min_margin_m']>0
    assert np.max(abs(curve(p['times'][-1],1)))<1e-8
    assert np.max(abs(np.array(p['path'])[:,:7]-q[:7]))<1e-9
