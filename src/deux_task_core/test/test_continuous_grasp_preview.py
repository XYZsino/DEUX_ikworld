from types import SimpleNamespace
import numpy as np
from deux_task_core.continuous_grasp_preview import plan
class Robot:
 velocity=np.ones(28)*2
 acceleration=np.ones(28)*3
 limits=np.tile([-2.,2.],(28,1))
 ik_position_tolerance_m=.03
 ik_rotation_tolerance_rad=.35
 def ee_fk(self,side,q):
  t=np.eye(4);t[:3,3]=q[14:17];return t
 def clearance(self,q,*args):
  if np.any(q<self.limits[:,0]) or np.any(q>self.limits[:,1]):raise ValueError('joint limit')
  return 1.
def inputs():
 r=Robot();p=SimpleNamespace(robot=r,config=dict(grasp_object_from_right_ee=np.eye(4).tolist(),obstacles=[],collision_margin=0.))
 q=np.zeros(28);q[14:17]=[0,.05,.16];v=np.zeros(28);v[15]=.05
 return p,q,v
def test_handoff_closure_and_independent_dynamics():
 p,q,v=inputs();result=plan(p,q,v,np.eye(4),[0,.05,0,0],1.6)
 assert result['valid'],result['reason']
 assert np.allclose(result['velocities'][0],v)
 assert abs(result['terminal_velocity'][15]-.05)<.005
 assert result['requires_continuous_successor'] and result['close_s']==.5
 assert result['acceleration_ratio']<=1 and result['velocity_ratio']<=1
 assert result['max_position_error_m']<.03
 assert np.allclose(q[14:17],[0,.05,.16])
def test_acceleration_limiting_does_not_hide_cartesian_failure():
 p,q,v=inputs();p.robot.acceleration=np.ones(28)*.01
 result=plan(p,q,v,np.eye(4),[0,.05,0,0],1.2)
 assert not result['valid'] and result['reason']=='position tracking residual'
 assert result['acceleration_ratio']<=1
