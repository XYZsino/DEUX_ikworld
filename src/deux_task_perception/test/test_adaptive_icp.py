import numpy as np
import pytest
from scipy.spatial import cKDTree
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from deux_task_msgs.msg import DetectionFrame,BoundingBox
from deux_task_perception.icp_estimator import IcpEstimator,ICP_DEFAULTS
from deux_task_perception.yaw_icp import fit_icp
from deux_task_perception.perception_core import rot,downsample

def test_first_frame_only_fallback_even_after_lease_change(monkeypatch):
 import deux_task_perception.icp_estimator as mod
 c=dict(ICP_DEFAULTS,reference_frame='base_link',base_frame='base_link',width=64,height=48,icp_adaptive_search=True,icp_initial_angles=12,icp_coarse_point_limit=400)
 points=np.ones((100,3));est=IcpEstimator(c,(points,points,'test'));calls=[]
 monkeypatch.setattr(mod,'roi_points',lambda *a,**kw:points)
 def fake(*a,**kw):
  calls.append(kw);t=np.eye(4);t[:3,3]=[.1,.2,.3]
  return dict(status='fitted',T_reference_model=t.tolist(),trimmed_rmse_m=.03,target_coverage_10mm=.2,seed_count=len(kw.get('initial_yaws',[])) or kw['initial_angles'])
 monkeypatch.setattr(mod,'fit_icp',fake)
 def frame(track,seq):
  h=Header(stamp=Time(sec=seq),frame_id='camera');d=Image(header=h,width=64,height=48,encoding='32FC1',step=64*4,data=np.ones((48,64),dtype='<f4').tobytes());info=CameraInfo(header=h,width=64,height=48)
  return DetectionFrame(header=h,depth=d,depth_info=info,sequence=seq,request_id='lease'+str(seq),boxes=[BoundingBox(class_name='s1',track_id=track,confidence=.95,xyxy=[1.,1.,20.,20.])])
 est.estimate((frame('a',1),np.eye(4),np.eye(4),(1,1)))
 assert len(calls)==2 and calls[0]['initial_angles']==6 and calls[1]['initial_angles']==12
 assert 'coarse_point_limit' not in calls[1] # full original fallback
 est.estimate((frame('a',2),np.eye(4),np.eye(4),(1,2)))
 assert len(calls)==3 and len(calls[-1]['initial_yaws'])==3
 assert not est.last_timing['search_attempts'][0]['first_frame']
 est.estimate((frame('b',3),np.eye(4),np.eye(4),(1,3)))
 assert len(calls)==5 # new identity gets its own first-frame fallback

def test_local_search_final_quality_uses_full_point_cloud():
 rng=np.random.default_rng(9);m=rng.uniform([-.08,-.04,-.03],[.08,.04,.03],(3600,3));n=m/np.linalg.norm(m,axis=1)[:,None]
 yaw=.4;t=np.array([.2,-.1,.4]);points=m[::3]@rot(yaw).T+t
 result=fit_icp(points,m,n,np.array([0,0,1.]),initial_yaws=[.35,.18,.52],initial_translation=t+[.005,0,0],coarse_point_limit=400,coarse_rank_reduced=True)
 assert result['seed_count']==3 and result['coarse_fit_points']==400 and result['fit_points']>400
 pose=np.array(result['T_reference_model']);dist=cKDTree(m[::3]).query((downsample(points)-pose[:3,3])@pose[:3,:3])[0]
 assert result['target_coverage_10mm']==pytest.approx(np.mean(dist<.01))
 assert result['trimmed_rmse_m']<.003
