import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from joycon_mobile_lift_logic import Control


def feed(c,s,t,button=False,x=0.,y=0.,turbo=False):
    a=[0.]*6;b=[0]*11
    ix,iy,it,ib=(0,1,4,4) if s=='left' else (2,3,5,6)
    a[ix]=x;a[iy]=y;a[it]=-1. if turbo else 1.;b[ib]=int(button)
    c.feed(s,a,b,t)


def enabled(s):
    c=Control();feed(c,s,0);feed(c,s,.01,button=True);return c


def test_startup_and_first_pressed_frame_do_not_enable():
    c=Control();feed(c,'right',0,button=True,y=1)
    assert c.output(0)==((0.,0.,0.),0)


def test_translation_diagonal_and_other_buttons_ignored():
    c=enabled('right');feed(c,'right',.02,x=1,y=1)
    assert c.output(.02)==((.4,.4,0.),0)
    c=Control();c.feed('right',[0.]*6,[1,1,1,1,0,0,0],0)
    c.feed('right',[0.]*6,[1,1,1,1,0,0,0],.1)
    assert not any(c.enabled.values())


def test_neutral_before_motion():
    c=Control();feed(c,'right',0,y=1);feed(c,'right',.1,button=True,y=1)
    assert c.output(.1)[0]==(0.,0.,0.)
    feed(c,'right',.2);feed(c,'right',.3,y=1)
    assert c.output(.3)[0][0]==.4


def test_rotation_diagonal_and_lift_hysteresis():
    c=enabled('left');feed(c,'right',0);feed(c,'right',.01,button=True)
    feed(c,'left',.02,x=1)
    assert c.output(.02)==((0.,0.,.8),0)
    feed(c,'left',.03,x=1,y=1);assert c.output(.03)==((0.,0.,0.),0)
    feed(c,'left',.04,y=.7);assert c.output(.04)[1]==1
    feed(c,'left',.05,y=.5);assert c.output(.05)[1]==1
    feed(c,'left',.06,y=.3);assert c.output(.06)[1]==0
    feed(c,'left',.07,y=-1);assert c.output(.07)[1]==2


def test_timeout_rejoin_and_estop_require_reenable():
    c=enabled('left');feed(c,'left',.02,y=1);assert c.output(.02)[1]==1
    assert c.output(.6)[1]==0
    feed(c,'left',.7,button=True,y=1);assert c.output(.7)[1]==0
    c=enabled('right');c.emergency(True);c.emergency(False)
    feed(c,'right',.1,y=1);assert c.output(.1)[0]==(0.,0.,0.)


def test_malformed_input_stops_both():
    for a,b in [([float('nan')]*6,[0]*11),([0.]*2,[0]*11),([0.]*6,[])]:
        c=enabled('right');c.feed('left',a,b,.02)
        assert c.output(.02)==((0.,0.,0.),0)


def test_turbo_requires_both_fresh():
    c=enabled('right');feed(c,'right',.1,y=1,turbo=True);feed(c,'left',.1,turbo=True)
    assert c.output(.1)[0][0]==1.
    feed(c,'right',.65,y=1,turbo=True)
    # Rejoined right also disables until toggled.
    assert c.output(.65)[0][0]==0.


def test_stale_side_only_disables_its_function():
    c=enabled('right');feed(c,'left',0);feed(c,'left',.01,button=True)
    feed(c,'right',.4,y=1);assert c.output(.6)==((.4,0.,0.),0)
    assert not c.enabled['left']


def test_plus_controls_rotation_without_minus_and_off_stops_rotation():
    c=enabled('right');feed(c,'left',.02);feed(c,'left',.03,x=1)
    assert c.output(.03)==((0.,0.,.8),0)
    feed(c,'left',.04,y=1);assert c.output(.04)[1]==0
    feed(c,'left',.05,x=1);feed(c,'right',.05)
    feed(c,'right',.06,button=True)
    assert c.output(.06)==((0.,0.,0.),0)


def test_minus_controls_only_lift_and_plus_off_keeps_lift():
    c=enabled('left');feed(c,'left',.02,x=1)
    assert c.output(.02)==((0.,0.,0.),0)
    feed(c,'left',.03,y=1);assert c.output(.03)[1]==1
    feed(c,'right',.03);feed(c,'right',.04,button=True)
    feed(c,'right',.05);feed(c,'right',.06,button=True)
    assert c.output(.06)==((0.,0.,0.),1)


def test_rotation_waits_for_left_neutral_on_plus_enable_and_reconnect():
    c=Control();feed(c,'left',0,x=1);feed(c,'right',0)
    feed(c,'right',.01,button=True)
    assert c.output(.01)[0][2]==0.
    feed(c,'left',.02);feed(c,'left',.03,x=1)
    assert c.output(.03)[0][2]==.8
    feed(c,'right',.4);assert c.output(.6)[0][2]==0.
    feed(c,'left',.61,x=1);assert c.output(.61)[0][2]==0.
    feed(c,'left',.62);feed(c,'left',.63,x=1)
    assert c.output(.63)[0][2]==.8
