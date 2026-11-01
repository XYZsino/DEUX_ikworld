import numpy as np
from scipy.spatial.transform import Rotation
from types import SimpleNamespace
from deux_task_core.position_first_preview import desired,integrate,rotation_progress

def test_slow_approach_rotation_and_smooth_handoff():
 initial=np.eye(4);initial[:3,3]=[.2,.1,1.];obj=np.eye(4);obj[:3,3]=[.3,.1,.8];g=np.eye(4);g[:3,:3]=Rotation.from_euler('z',60,degrees=True).as_matrix();v=np.array([0,.05,0,0])
 for t in [0,.5,1.]:
  target,feed=desired(initial,obj,v,g,.2,1.,4.,t)
  angle=Rotation.from_matrix(target[:3,:3]).magnitude()
  assert angle<=np.radians(15)*t+1e-9
  assert np.linalg.norm(feed[3:])<=np.radians(15)+1e-9
  if t>0:assert angle>0
 end,feed=desired(initial,obj,v,g,.2,1.,4.,1.)
 assert np.allclose(feed[:3],v[:3])
 after,feed2=desired(initial,obj,v,g,.2,1.,4.,1.+1e-6)
 assert np.linalg.norm(end[:3,3]-after[:3,3])<1e-6
 assert np.linalg.norm(feed-feed2)<1e-5
 final,_=desired(initial,obj,v,g,.2,1.,4.,5.)
 assert np.allclose(final[:3,:3],g[:3,:3])
 assert np.allclose(final[:3,3],[.3,.4,1.])
 rates=[np.linalg.norm(desired(initial,obj,v,g,.2,1.,4.,t)[1][3:]) for t in np.linspace(1,5,100)]
 assert max(rates)<=np.radians(30)+1e-8

class Robot:
 velocity=np.ones(28)*2;acceleration=np.ones(28)*3;limits=np.tile([-2.,2.],(28,1));ik_position_tolerance_m=.03;ik_rotation_tolerance_rad=.35
 def ee_fk(self,side,q):
  t=np.eye(4);t[:3,3]=q[14:17];t[:3,:3]=Rotation.from_rotvec(q[17:20]).as_matrix();return t
 def clearance(self,q,*args):
  if np.any(q<self.limits[:,0]) or np.any(q>self.limits[:,1]):raise ValueError('joint limit')
  return 1.
def test_measured_start_and_no_reset_between_stages():
 r=Robot();q=np.zeros(28);q[14:17]=[.2,0,1.];obj=np.eye(4);obj[:3,3]=[.25,0,.8];g=np.eye(4);g[:3,:3]=Rotation.from_euler('z',10,degrees=True).as_matrix()
 c=dict(approach_height=.2,obstacles=[],collision_margin=0.)
 result=integrate(r,c,q,obj,np.array([0,.02,0,0]),g,1.2,1.2,False)
 assert result['valid'],result['reason'];assert np.allclose(result['path'][0],q)
 assert result['acceleration_ratio']<=1;assert result['velocity_ratio']<=1
 assert abs(result['terminal_velocity'][15]-.02)<.003
 assert np.allclose(np.array(result['path'])[:,:14],q[:14])
def test_low_acceleration_rejects_cartesian_miss():
 r=Robot();r.acceleration=np.ones(28)*.001;q=np.zeros(28);obj=np.eye(4);obj[:3,3]=[.3,0,0]
 result=integrate(r,dict(approach_height=0,obstacles=[],collision_margin=0),q,obj,np.zeros(4),np.eye(4),1,1,False)
 assert not result['valid'];assert result['reason']=='position tracking residual'

def test_short_tracking_window_does_not_claim_full_alignment():
 r=Robot();q=np.zeros(28);q[14:17]=[.2,0,1.];obj=np.eye(4);obj[:3,3]=[.23,0,.8];g=np.eye(4);g[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
 result=integrate(r,dict(approach_height=.2,obstacles=[],collision_margin=0),q,obj,np.array([0,.01,0,0]),g,1.2,6.,False,tracking_window_s=1.)
 assert result['valid'],result['reason'];assert abs(result['times'][-1]-2.2)<1e-8
 actual=r.ee_fk('right',np.array(result['path'][-1]));remaining=Rotation.from_matrix(g[:3,:3]@actual[:3,:3].T).magnitude()
 assert remaining>np.radians(15);assert abs(result['terminal_velocity'][15]-.01)<.003


def test_rotation_caps_continuity_and_no_overshoot():
 for angle in np.radians([0,.5,10,60,150]):
  values=np.array([rotation_progress(angle,t,1.2) for t in np.linspace(0,12,1201)])
  assert np.all(values[:,0]>=-1e-9) and np.all(values[:,0]<=angle+1e-9)
  assert np.all(np.diff(values[:,0])>=-1e-9)
  assert abs(values[-1,0]-angle)<1e-8 and abs(values[-1,1])<1e-8
  assert np.max(values[:121,1])<=np.radians(15)+1e-9
  assert np.max(values[:,1])<=np.radians(30)+1e-9
  left=np.array(rotation_progress(angle,1.2-1e-6,1.2));right=np.array(rotation_progress(angle,1.2+1e-6,1.2))
  assert np.max(abs(left-right))<1e-5
