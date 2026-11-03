"""No nodes or hardware clients are constructed."""
import sys,time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
sys.path.insert(0,str(Path('tools').resolve()))
import moving_upper_trial as m
from deux_task_core.tracking_policy import validate_policy

def fixture(monkeypatch):
 monkeypatch.setattr(m,'seconds',float)
 monkeypatch.setattr(m,'pose_matrix',lambda p:np.eye(4))
 clock=[100.]
 d=NS(track_id='A',velocity_valid=True,velocity=NS(linear=NS(x=.0,y=-.04,z=0.),angular=NS(z=0.)),pose=NS(pose=None))
 n=NS(calibration={'valid':True},cal_received=time.monotonic(),track={'tracking':True,'target_id':'A'},track_received=time.monotonic(),objects=NS(header=NS(stamp=100.),detections=[d]),now=lambda:clock[0],config={'tracking_grasp_policy':{'prediction_hold_s':1.}})
 return n,clock,d

def test_hold_acquisition_deadline_and_no_start_on_stale(monkeypatch):
 n,t,d=fixture(monkeypatch)
 assert m.MovingTrial.tracked_object(n)[3]==100.
 t[0]=100.8
 with pytest.raises(ValueError):m.MovingTrial.tracked_object(n)
 assert m.MovingTrial.tracked_object(n,True)[3]==100.
 assert n.observation_held
 t[0]=101.
 assert m.MovingTrial.tracked_object(n,True)[3]==100.
 t[0]=101.001
 with pytest.raises(ValueError):m.MovingTrial.tracked_object(n,True)

def test_invalid_velocity_sample_does_not_destroy_good_cache(monkeypatch):
 n,t,d=fixture(monkeypatch);m.MovingTrial.tracked_object(n)
 d.velocity_valid=False;n.objects.header.stamp=100.2;t[0]=100.3
 assert m.MovingTrial.tracked_object(n,True)[3]==100.
 d.velocity_valid=True
 assert m.MovingTrial.tracked_object(n,True)[3]==100.2
 assert not n.observation_held

@pytest.mark.parametrize('change',['exit','identity','calibration','velocity'])
def test_hold_does_not_override_guards(monkeypatch,change):
 n,t,d=fixture(monkeypatch);m.MovingTrial.tracked_object(n);t[0]=100.3
 if change=='exit':n.track['tracking']=False
 if change=='identity':n.track['target_id']='B'
 if change=='calibration':n.calibration['valid']=False
 if change=='velocity':d.velocity.linear.y=.3
 with pytest.raises(ValueError):m.MovingTrial.tracked_object(n,True)

def test_close_without_reobservation_within_budget(monkeypatch,tmp_path):
 import asyncio,json
 import descent_trial as module
 from builtin_interfaces.msg import Time
 monkeypatch.setattr(module,'roi_contains',lambda *a:True)
 monkeypatch.setattr(module,'ROOT',tmp_path)
 policy=json.loads(Path('calibration/tracking_grasp_policy.json').read_text())
 target=np.eye(4);target[:3,3]=[.58,-.1,.97];fk=target.copy();fk[2,3]+=.01
 sends=[];clock=[100.4]
 n=NS(config={'tracking_grasp_policy':policy,'home_joints':[0.]*28,'approach_height':.08},idx=list(range(7))+list(range(14,21)),passive_left=np.zeros(7),robot=NS(ee_fk=lambda *args:fk),arm=NS(header=NS(stamp=Time(sec=100))),now=lambda:clock[0],progressive_closed=False,close_started=None,pregrasp_started=99.,progressive_events=[],close_commands=[],right_trial_hand=NS(send=sends.append),observation_held=True)
 n.config_version='test'
 plan=dict(start_at=100.,times=[0.,5.],path=np.zeros((2,28)).tolist(),velocities=np.zeros((2,28)).tolist(),approach_s=.1,brake_start_s=5.,grasp=np.eye(4),geometry={},workspace_follow=True,descent_profile={'start':0.,'duration':.1,'height0':.01,'height1':.01,'velocity0':0.})
 async def tick(ok=True):await module.GraspTrial.policy_tick(n,None,plan,np.zeros(14),1.,ok,(target,np.zeros(4),100.) if ok else None)
 asyncio.run(tick());assert sends[-1]==1. and n.close_started==100.4
 clock[0]=100.91;asyncio.run(tick());assert n.progressive_closed
 assert all(x['prediction_hold'] for x in n.close_commands)
 n.progressive_closed=False
 with pytest.raises(ValueError,match='observation lost'):asyncio.run(tick(False))
