from types import SimpleNamespace
import numpy as np
import pytest
from deux_task_core import intercept_preview as p
from deux_task_core.prediction_audit import PredictionAudit

def config():
 return dict(home_joints=[0.]*28,descend_distance_m=.06,approach_height=.16275,hand_close_duration_s=.5,hand_open=[0.]*14,hand_closed=[1.]*14,grasp_object_from_right_ee=np.eye(4).tolist())
class Robot:
 limits=np.tile([-2.,2.],(28,1))
 def ee_fk(self,side,q):
  result=np.eye(4);result[:3,3]=q[:3];return result

def test_candidates_full_descent_and_closing_follow(monkeypatch):
 monkeypatch.setattr(p,'_CACHE',SimpleNamespace(robot=Robot()));monkeypatch.setattr(p.time,'monotonic',lambda:10.)
 calls=[]
 def plan(trial,stage,q,obj,v,lead):
  duration=trial.config['conveyor_tracking']['durations'][stage];calls.append(stage)
  assert trial.config['hand_closed']==[0.]*14
  target=p.forecast(obj,v,lead+duration)@np.asarray(trial.config['grasp_object_from_right_ee'])
  if stage=='Approach':target[2,3]+=trial.config['approach_height']
  end=np.array(q);end[:3]=target[:3,3]
  return dict(times=np.array([0.,duration]),path=np.array([q,end]))
 monkeypatch.setattr(p,'plan_conveyor',plan)
 c=config();out=p.solve(c,[0.]*28,np.eye(4),[.01,0,0,0],.2,10.,100.)
 assert out['feasible'] and out['candidate_count']==8
 assert out['grasp_pose'][2][3]==0
 assert out['approach_pose'][2][3]==pytest.approx(.16275)
 assert out['close_at_s']-out['contact_at_s']==pytest.approx(.5)
 assert out['close_pose'][0][3]-out['grasp_pose'][0][3]==pytest.approx(.005)
 assert c['hand_closed']==[1.]*14 and c['descend_distance_m']==.06
 assert [x['stage'] for x in out['trace']]==['Approach','Descend','Close']

def test_failure_no_false_point(monkeypatch):
 monkeypatch.setattr(p,'_CACHE',SimpleNamespace(robot=Robot()))
 def fail(*a):raise ValueError('unreachable')
 monkeypatch.setattr(p,'plan_conveyor',fail)
 out=p.solve(config(),[0.]*28,np.eye(4),[0]*4,0.,p.time.monotonic(),100.)
 assert not out['feasible'] and 'grasp_pose' not in out and out['candidate_count']==8

def test_audit_same_track_interpolation_and_missing():
 a=PredictionAudit();a.pending=[dict(target_id='a',stage='contact',at=1.,expected=[1,0,0])]
 assert a.observe('other',.9,[0,0,0])==[]
 assert a.observe('a',.9,[.9,0,0])==[]
 result=a.observe('a',1.1,[1.1,0,0]);assert result[0]['matched'] and result[0]['error_m']<1e-9
 a.pending=[dict(target_id='a',stage='close',at=2.,expected=[2,0,0])]
 result=a.observe('b',4.1,[0,0,0]);assert not result[0]['matched']
