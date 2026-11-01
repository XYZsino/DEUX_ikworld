"""Offline policy tests; no ROS nodes or hardware clients constructed."""
import sys,json,copy,asyncio
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np,pytest
sys.path.insert(0,str(Path('tools').resolve()))
from deux_task_core.tracking_policy import *
from deux_task_core.position_first_preview import desired
from progressive_descent import next_profile,height_state

@pytest.fixture
def policy():return validate_policy(json.loads(Path('calibration/tracking_grasp_policy.json').read_text()))

def test_workspace_boundaries_and_hysteresis(policy):
 o=np.eye(4);o[:3,3]=[.52,-.1,.97];v=np.zeros(4)
 assert not follow_admitted(policy,o,v,np.eye(4),0.,False)
 assert follow_admitted(policy,o,v,np.eye(4),0.,True)
 o[0,3]=.519;assert not follow_admitted(policy,o,v,np.eye(4),0.,True)
 o[0,3]=.526;assert follow_admitted(policy,o,v,np.eye(4),0.,False)

def test_forecast_exit_prevents_descent(policy):
 o=np.eye(4);o[:3,3]=[.58,-.20,.97];v=np.array([0.,-.1,0.,0.])
 assert forecast_inside(policy,o,v,np.eye(4),0.,.4)
 assert not forecast_inside(policy,o,v,np.eye(4),0.,.6)

def test_outside_only_x_rotation(policy):
 initial=np.eye(4);initial[:3,3]=[.58,-.14,1.05];o=np.eye(4);o[:3,3]=[.70,.2,.96];v=np.array([.01,-.05,0.,0.])
 t,feed=desired(initial,o,v,np.eye(4),.08,1.,10.,2.,0.,policy,False)
 assert np.allclose(t[:3,3],[policy['xy_max_m'][0],policy['standby_y_m'],1.05]);assert np.allclose(feed[:3],0)
 t0,_=desired(initial,o,v,np.eye(4),.08,1.,10.,0.,0.,policy,False)
 np.testing.assert_allclose(t0,initial)

def test_inside_xyz_tracking(policy):
 initial=np.eye(4);o=np.eye(4);o[:3,3]=[.58,.0,.96];v=np.array([0.,-.05,0.,0.])
 t,feed=desired(initial,o,v,np.eye(4),.08,1.,10.,2.,0.,policy,True)
 assert np.allclose(t[:3,3],[.58,-.1,1.04]);assert np.allclose(feed[:3],v[:3])

@pytest.mark.parametrize('xy,h,remaining,inside,ok',[(.019,.014,.6,True,True),(.021,.014,.6,True,False),(.01,.016,.6,True,False),(.01,-.001,.6,True,False),(.01,.01,.5,True,False),(.01,.01,.6,False,False)])
def test_split_closure_gate(policy,xy,h,remaining,inside,ok):assert closure_ok(policy,xy,h,remaining,inside,True)==ok

def test_descent_monotonic_and_terminal_velocity():
 p=next_profile({},0.,.8,.08,True)
 x=np.array([height_state(p,t,.08) for t in np.linspace(0,.8,51)])
 assert np.max(np.diff(x[:,0]))<1e-10 and x[-1,0]==pytest.approx(.032)
 assert x[-1,1]==0

def test_first_seed_comparison_only(policy,monkeypatch):
 import initial_seed_trial as m
 m._CACHE.clear()
 class Robot:
  limits=np.tile([-3.,3.],(28,1));tracking_joint_reserve_rad=.01
  def __init__(self):self.calls=0
  def ee_fk(self,*args):return np.eye(4)
  def ik(self,side,target,seed):self.calls+=1;return seed
 r=Robot();monkeypatch.setattr(m,'backend_robot',lambda config:r);c=dict(tracking_grasp_policy=policy,approach_height=.08,_tracking_target_id='A',_seed_config_version='v')
 args=(r,c,np.zeros(28),np.eye(4),np.zeros(4),np.eye(4),1.)
 _,first=m.references(*args);_,again=m.references(*args)
 assert first and not again and r.calls==2
 c['_tracking_target_id']='B';m.references(*args);assert r.calls==4

def test_runtime_preopen_and_close(policy,monkeypatch,tmp_path):
 import descent_trial as m
 monkeypatch.setattr(m,'roi_contains',lambda *a:True)
 monkeypatch.setattr(m,'ROOT',tmp_path)
 from builtin_interfaces.msg import Time
 target=np.eye(4);target[:3,3]=[.58,-.1,.97];actual=target.copy();actual[2,3]+=.01
 sends=[];clock=[100.]
 n=NS(config={'tracking_grasp_policy':policy,'home_joints':[0.]*28,'approach_height':.08},idx=list(range(7))+list(range(14,21)),passive_left=np.zeros(7),robot=NS(ee_fk=lambda *args:actual),arm=NS(header=NS(stamp=Time(sec=100))),now=lambda:clock[0],progressive_closed=False,close_started=None,pregrasp_started=None,progressive_events=[],close_commands=[],right_trial_hand=NS(send=sends.append))
 n.config_version='test'
 plan=dict(start_at=100.,times=[0.,5.],path=np.zeros((2,28)).tolist(),velocities=np.zeros((2,28)).tolist(),approach_s=.1,brake_start_s=5.,grasp=np.eye(4),geometry={},workspace_follow=True,descent_profile={'start':0.,'duration':.1,'height0':.01,'height1':.01,'velocity0':0.})
 async def tick():await m.GraspTrial.policy_tick(n,None,plan,np.zeros(14),1.,True,(target,np.zeros(4),100.))
 asyncio.run(tick());assert sends==[.4] and n.close_started is None
 clock[0]=100.51;asyncio.run(tick());assert sends[-1]==1. and n.close_started==clock[0]
 clock[0]=101.02;asyncio.run(tick());assert n.progressive_closed


def test_repeated_descent_reaches_zero_without_reversal():
    previous={}
    for splice in [0.,.4,.8,1.2,1.6]:
        profile=next_profile(previous,splice,.8,.08,True)
        h=np.array([height_state(profile,t,.08)[0] for t in np.linspace(splice,splice+.8,101)])
        assert h.min()>=-1e-10 and np.max(np.diff(h))<=1e-10
        previous={'descent_profile':profile}
    assert h[-1]==0.


def test_left_drift_does_not_abort_right_only_gate(tmp_path,monkeypatch):
 import descent_trial as m
 monkeypatch.setattr(m,'OUT',tmp_path)
 actual=np.zeros(14);actual[:7]=np.deg2rad(20.)
 n=NS(estop=False,now=lambda:100.,hardware_gate=lambda:None,check_model_version=lambda:None,external=0.,
      measured=lambda:actual.copy(),robot=NS(limits=np.tile([-1.,1.],(28,1)),joint_limit_tolerance=.001),passive_left=np.zeros(7))
 checked=m.GraspTrial.gate(n)
 np.testing.assert_allclose(checked[:7],0.)
 np.testing.assert_allclose(checked[7:],actual[7:])
 actual[7]=2.
 with pytest.raises(RuntimeError,match='right measured joint limit'):m.GraspTrial.gate(n)
