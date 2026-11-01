import json,sys,numpy as np,pytest
sys.path.insert(0,'tools')
from intercept_window import contains,arrival_times,polygon,compute_intercept
from deux_task_core.position_first_preview import desired
from deux_task_core.configuration import load_configuration
from right_only_trial import planning_robot
from progressive_descent import target,close_eligible

def zone():
 return dict(origin_base_m=[.6,0.,.9],direction_xy=[0.,-1.],along_m=[-.25,.25],half_width_m=.08,z_range_m=[.85,.95])
def test_zone_and_future_window():
 z=zone();obj=np.eye(4);obj[:3,3]=[.6,.3,.9];v=np.array([0.,-.1,0.,0.])
 times=arrival_times(z,obj,v,3.)
 assert times and all(.8<=t<=2.5 for t in times)
 for t in times:
  assert contains(z,[[.6,.3-.1*t,.9],[.6,.3-.1*(t+3),.9]])
 assert not contains(z,[[.7,0.,.9]])
 with pytest.raises(ValueError):arrival_times(z,obj,[0,.1,0,0],3.)
 assert len(polygon(z))==4

def test_time_consistent_tracking_and_feedforward():
 initial=np.eye(4);obj=np.eye(4);obj[:3,3]=[.6,.3,.9];v=np.array([0.,-.1,0.,0.]);g=np.eye(4)
 for t in [1.,1.5,2.]:
  p,feed=desired(initial,obj,v,g,.1,1.,10.,t,lookahead=0.)
  assert np.allclose(p[:3,3],[.6,.3-.1*t,1.])
  assert np.allclose(feed[:3],v[:3])
  p2,_=target(initial,obj,v,g,.1,1.,t,lookahead=0.)
  assert np.allclose(p,p2)
 # Legacy lookahead remains opt-in/default for unrelated preview users.
 p,_=desired(initial,obj,v,g,.1,1.,10.,2.)
 assert np.isclose(p[1,3],0.)

def test_closure_still_requires_position_and_time():
 assert close_eligible(.029,1,2,True,True)
 assert close_eligible(.05,1,2,True,True)
 assert not close_eligible(.05001,1,2,True,True)
 assert not close_eligible(.04,1,2,False,True)
 assert not close_eligible(.04,1,2,True,False)
 assert not close_eligible(float("nan"),1,2,True,True)
 assert not close_eligible(.029,1,1.4,True,True)

def test_full_path_with_real_robot_synthetic_target():
 c,_=load_configuration('src/deux_task_bringup/config/installation.yaml')
 q=np.array(json.load(open('.setup/selected-ready.json'))['chosen']['new_home'])
 r=planning_robot(c);ee=r.ee_fk('right',q);obj=ee.copy();obj[:3,3]+=[0,.04,-.03]
 z=zone();z.update(origin_base_m=(obj[:3,3]-[0,.04,0]).tolist(),z_range_m=[.99,1.12])
 c.update(intercept_window=z,tracking_lookahead_s=0.,tracking_rotation_weight=.05,progressive_descent=True,approach_height=.03,grasp_object_from_right_ee=np.eye(4).tolist())
 c['_stationary_left_arm']=q[:7].tolist()
 # Use actual camera/ROI geometry for the synthetic object.
 old=json.load(open('.setup/moving-upper-trial/progressive-repeat-1793466501696689167/1-result.json'))
 p=compute_intercept(c,q,obj,[0,-.02,0,0],0.,old['geometry'])
 assert p['intercept']['full_path_validated']
 assert p['intercept']['min_joint_margin_deg']>=3
 assert p['intercept']['close_position_error_m']<=.03
 assert p['intercept']['close_position_policy']=='warning_only'
 assert p['intercept']['live_close_tolerance_m']==.05
 assert p['tracking_lookahead_s']==0

def test_window_overlay_uses_base_to_camera_and_hides_unvalidated_meet():
 sys.path.insert(0,'src/deux_task_gui/test')
 from test_rgb_view import fixture
 from deux_task_gui.rgb_view import intercept_pixels,render_rgb
 obs,_=fixture()
 window=dict(frame='base_link',polygon=[[-.1,-.1,1.],[.1,-.1,1.],[.1,.1,1.],[-.1,.1,1.]],encounter_position=[0,0,1],validated=False)
 assert intercept_pixels(obs,window).shape==(4,2)
 window['validated']=True
 assert np.allclose(intercept_pixels(obs,window)[4],[48,36])
 _,meta=render_rgb(obs,None,'',intercept_window=window)
 assert meta['intercept_window_visible']
 window['frame']='wrong_frame'
 assert intercept_pixels(obs,window) is None

def test_j2_slowdown_respects_same_reserve_as_path_validator():
 from deux_task_core.position_first_preview import bounded_joint_velocity
 from scipy.interpolate import CubicHermiteSpline
 c,_=load_configuration('src/deux_task_bringup/config/installation.yaml')
 c['intercept_window']={'min_nominal_joint_margin_deg':3.}
 r=planning_robot(c)
 for side in [-1,1]:
  q=np.array(c['home_joints']);j=15
  boundary=r.limits[j,0] if side<0 else r.limits[j,1]
  q[j]=boundary-side*np.deg2rad(10)
  qd=np.zeros(7);path=[q[j]];vel=[0.]
  for _ in range(150):
   wanted=np.zeros(7);wanted[1]=side*r.velocity[j]
   wanted=bounded_joint_velocity(r,q,wanted)
   new=qd+np.clip(wanted-qd,-r.acceleration[14:21]*.95*.02,r.acceleration[14:21]*.95*.02)
   q[14:21]+=.5*(qd+new)*.02;qd=new
   path.append(q[j]);vel.append(qd[1])
  s=CubicHermiteSpline(np.arange(151)*.02,path,vel)
  roots=s.derivative().roots(extrapolate=False)
  values=s(np.r_[np.arange(151)*.02,roots[np.isfinite(roots)]])
  assert values.min()>=r.limits[j,0]+np.deg2rad(3)
  assert values.max()<=r.limits[j,1]-np.deg2rad(3)

def test_approach_first_checks_executed_path_and_defers_descent(monkeypatch):
 import async_upper
 c,_=load_configuration('src/deux_task_bringup/config/installation.yaml')
 d=json.load(open('.setup/moving-upper-trial/latest-compute-input.json'))
 c.update(intercept_window=json.load(open('calibration/intercept_window.json')),intercept_approach_first=True,tracking_lookahead_s=0.,tracking_rotation_weight=.05,progressive_descent=True)
 c['_stationary_left_arm']=d['q'][:7]
 p=compute_intercept(c,d['q'],d['object_pose'],d['velocity'],1.2,d['geometry'])
 assert p['intercept']['upper_path_validated']
 assert not p['intercept']['full_path_validated']
 assert np.max(abs(np.array(p['velocities'][-1])))<1e-8
 async_upper.validate(c,p)
 def blocked(*args,**kwargs):raise ValueError('test collision')
 monkeypatch.setattr(async_upper,'validate',blocked)
 with pytest.raises(ValueError,match='test collision'):
  compute_intercept(c,d['q'],d['object_pose'],d['velocity'],1.2,d['geometry'])

def test_height_gate_disabled_preserves_xy_and_finite_checks():
 z=zone();z['enforce_height_range']=False
 assert contains(z,[[.6,0.,.972]])
 assert not contains(z,[[.7,0.,.972]])
 assert not contains(z,[[.6,.3,.972]])
 assert not contains(z,[[.6,0.,float('nan')]])
 z['enforce_height_range']=True
 assert not contains(z,[[.6,0.,.972]])
