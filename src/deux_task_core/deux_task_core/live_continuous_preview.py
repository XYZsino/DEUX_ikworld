"""Live observations, hypothetical tracking state, no robot commands."""
import time
from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from .planning import Planner
from .model import pose
from .conveyor import forecast,tracking_joint_velocity
from .continuous_grasp_preview import plan
from .approach_screen import screen
_CACHE=None

def warm(config):
 global _CACHE
 _CACHE=Planner(config);q=np.array(config['home_joints']);_CACHE.robot.ik('right',_CACHE.robot.ee_fk('right',q),q);return True

def roi_contains(geometry,positions):
 cam=np.linalg.inv(np.asarray(geometry['camera']));points=np.asarray(positions)@cam[:3,:3].T+cam[:3,3]
 if np.any(points[:,2]<=.01):return False
 p=np.column_stack([points,np.ones(len(points))])@np.asarray(geometry['projection']).reshape(3,4).T;p=p[:,:2]/p[:,2,None]
 roi=geometry['roi'];poly=np.array(roi['points'])*[geometry['width']/roi['width'],geometry['height']/roi['height']]
 return all(cv2.pointPolygonTest(poly.astype(np.float32),tuple(map(float,xy)),True)>=2. for xy in p)

def solve(config,q,object_pose,velocity,age_at_submit,submitted_monotonic,acquired_s,geometry):
 global _CACHE
 if _CACHE is None:warm(config)
 began=time.monotonic();r=_CACHE.robot;q=np.array(q);obj=pose(object_pose);v=np.array(velocity);reference=np.array(config['home_joints']);reference[:14]=q[:14];reference[21:]=q[21:]
 base_grasp=pose(config['grasp_object_from_right_ee']);start_age=age_at_submit+(began-submitted_monotonic);start_obj=forecast(obj,v,start_age)
 # Preview-only angle policy: keep measured object pose unchanged, rotate grasp offset.
 minimum=float(config.get('preview_min_grasp_yaw_deg',0.))
 observed_yaw=float(np.degrees(np.arctan2(obj[1,0],obj[0,0]))%180.)
 chosen_yaw=float(np.clip(observed_yaw,minimum,180.-minimum))
 delta=chosen_yaw-observed_yaw
 base_grasp[:3,:3]=Rotation.from_euler('z',delta,degrees=True).as_matrix()@base_grasp[:3,:3]
 reports=[dict(observed_yaw_deg=observed_yaw,grasp_yaw_deg=chosen_yaw,grasp_yaw_offset_deg=delta,policy='preview only: clamp modulo180 yaw to 30..150')];lookahead=1.;actual=r.ee_fk('right',q)
 for flip in [0,180]:
  grasp=base_grasp.copy();grasp[:3,:3]=Rotation.from_euler('z',flip,degrees=True).as_matrix()@grasp[:3,:3]
  upper=forecast(start_obj,v,lookahead)@grasp;upper[2,3]+=config['approach_height']
  if not roi_contains(geometry,[start_obj[:3,3],forecast(start_obj,v,lookahead)[:3,3]]):reports.append(dict(flip=flip,reason='1초 예측점이 ROI 밖'));continue
  try:
   seed=r.ik('right',upper,reference,continuity_weight=.005)
   approach_screen=screen(r,config,q,seed,v)
   reports.append(dict(flip=flip,current_approach=approach_screen))
   incoming=tracking_joint_velocity(r,seed,v[:3],np.zeros(3))
  except ValueError as e:reports.append(dict(flip=flip,reason=str(e)));continue
  c=deepcopy(config);c['grasp_object_from_right_ee']=grasp.tolist();trial=SimpleNamespace(config=c,robot=r)
  for duration in [1.2,1.6,2.,2.4]:
   horizon=duration+.5
   if not roi_contains(geometry,[forecast(start_obj,v,t)[:3,3] for t in np.linspace(0,horizon,12)]):reports.append(dict(flip=flip,descent_s=duration,reason='닫힘 완료 전에 ROI 이탈 예상'));continue
   try:result=plan(trial,seed,incoming,start_obj,v,duration)
   except ValueError as e:reports.append(dict(flip=flip,descent_s=duration,reason=str(e)));continue
   reports.append(dict(flip=flip,descent_s=duration,valid=result['valid'],reason=result['reason'],max_position_error_m=result['max_position_error_m'],max_rotation_error_rad=result['max_rotation_error_rad']))
   if result['valid']:
    elapsed=time.monotonic()-began
    if elapsed>=duration:reports.append(dict(reason='계산 완료 전에 가상 접촉 시각 경과'));continue
    path=np.array(result['path']);ts=np.array(result['times']);contact_i=int(np.argmin(abs(ts-duration)));traces=[]
    for name,indices in [('Descend',np.flatnonzero(ts<=duration)),('Close',np.flatnonzero(ts>=duration))]:
     indices=indices[np.unique(np.r_[np.arange(0,len(indices),max(1,len(indices)//20)),len(indices)-1])]
     traces.append(dict(stage=name,times_s=(ts[indices]+start_age).tolist(),points=[result['ee_points'][i] for i in indices]))
    contact=r.ee_fk('right',path[contact_i]);closed=r.ee_fk('right',path[-1])
    return dict(feasible=True,conditional=True,preview_only=True,actual_execution_feasible=False,actual_reason='현재→고정 상부 경로: '+('통과' if approach_screen['passed'] else '실패')+'; 이동 대상 도착 시점은 미검증',current_approach=approach_screen,current_to_virtual_upper_m=float(np.linalg.norm(actual[:3,3]-upper[:3,3])),lookahead_s=lookahead,acquired_s=acquired_s,launch_at_s=acquired_s+start_age,contact_at_s=acquired_s+start_age+duration,close_at_s=acquired_s+start_age+horizon,object_pose=forecast(start_obj,v,duration).tolist(),close_object_pose=forecast(start_obj,v,horizon).tolist(),grasp_pose=contact.tolist(),close_pose=closed.tolist(),approach_pose=r.ee_fk('right',seed).tolist(),joint_start=q.tolist(),virtual_start=seed.tolist(),incoming_velocity=incoming.tolist(),terminal_velocity=result['terminal_velocity'],requires_continuous_successor=True,trace=traces,durations_s=dict(TrackLookahead=lookahead,Descend=duration,Close=.5),prediction_horizon_s=start_age+horizon,planning_ms=elapsed*1000,max_position_error_m=result['max_position_error_m'],acceleration_ratio=result['acceleration_ratio'],velocity_ratio=result['velocity_ratio'],reports=reports,candidate_count=len(reports),feasible_count=1,method='live observations; hypothetical aligned tracking + continuous descent/closure; no approach or execution')
   if time.monotonic()-began>1.5:break
  if time.monotonic()-began>1.5:break
 return dict(feasible=False,conditional=True,actual_execution_feasible=False,reason=reports[-1]['reason'] if reports else '유효 후보 없음',reports=reports,planning_ms=(time.monotonic()-began)*1000,candidate_count=len(reports),feasible_count=0,lookahead_s=1.,max_descent_s=2.4,close_s=.5)
