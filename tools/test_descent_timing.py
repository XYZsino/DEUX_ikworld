import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path('tools').resolve()))
from descent_timing import curve,estimate_delay,FeedbackTiming,validated_timing

def plan():
 ts=np.linspace(0,3,31);q=np.zeros((len(ts),28));q[:,14]=.1-.05*ts;v=np.zeros_like(q);v[:,14]=-.05
 return dict(start_at=100.,times=ts.tolist(),path=q.tolist(),velocities=v.tolist(),generation=1,brake_start_s=3.,object_at_start=np.eye(4).tolist(),velocity=[0.]*4,grasp=np.eye(4).tolist(),descent_profile=dict(start=0.,duration=2.,height0=.1,height1=0.,velocity0=0.),descent_timing=dict(pose_arm_rad=[0.]*7,completion_s=2.2,completion_within_validated_path=True))

def test_stamp_alignment_and_stationary_exclusion():
 p=plan();p['velocities']= (np.array(p['velocities'])*2).tolist();p['path']=(np.array(p['path'])*2).tolist();s=curve(p)
 assert estimate_delay(s,1.,s(.85))==pytest.approx(.15)
 p['path']=np.zeros((31,28)).tolist();p['velocities']=np.zeros((31,28)).tolist()
 assert estimate_delay(curve(p),1.,np.zeros(28)) is None

def test_validated_fk_duration_not_nominal():
 class Robot:
  def ee_fk(self,side,q):
   t=np.eye(4);t[2,3]=q[14];return t
 p=plan();meta=validated_timing(Robot(),p,dict(close_height_m=.015,close_duration_s=.5))
 assert meta['close_crossing_s']==pytest.approx(1.7)
 assert meta['completion_s']==pytest.approx(2.2)
 assert meta['completion_within_validated_path']
 p['brake_start_s']=1.
 assert validated_timing(Robot(),p,dict(close_height_m=.015,close_duration_s=.5))['completion_s'] is None

def test_learns_delay_and_corrects_remaining_height(tmp_path):
 p=plan();p['path']=(np.array(p['path'])*2).tolist();p['velocities']=(np.array(p['velocities'])*2).tolist();s=curve(p)
 m=FeedbackTiming(tmp_path/'delay.json','v');policy=dict(descent_speed_m_s=.06,close_height_m=.015,close_duration_s=.5,max_descent_m=.05)
 for t in np.linspace(.6,.9,7):
  out=m.update(p,s(t-.15),100+t,100+t+.1,.05-.02*(t-.15),policy)
 assert out['delay_measured'];assert out['delay_s']==pytest.approx(.15)
 assert out['feedback_age_s']==pytest.approx(.1)
 assert out['remaining_s']>=out['live_remaining_s']
 assert len(next(iter(FeedbackTiming(tmp_path/'delay.json','v').bins.values())))>=5
 n=FeedbackTiming(tmp_path/'delay.json','different-version');assert not n.bins
