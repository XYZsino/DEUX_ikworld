from right_only_trial import planning_robot
"""One GUI-approved, time-stamped moving upper approach. No descent/hand commands."""
import json,time,uuid,os
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from deux_task_msgs.msg import ObjectDetectionArray,TaskStatus,MotionPlan
from deux_task_msgs.action import MoveToReady
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.qos import qos_profile_sensor_data
from deux_task_runtime.common import pause,ros_time,seconds,pose_matrix,cancel_children
from deux_task_runtime.motion_review import preview,Ticket
from deux_task_runtime.trajectory import validate_limits,sample
from deux_task_core.model import RobotModel
from deux_task_core.conveyor import forecast
from deux_task_core.position_first_preview import integrate
from deux_task_core.live_continuous_preview import roi_contains
from deux_task_core.conveyor_screen import clearance
from ready_arm_trial import Trial,ROOT
from async_upper import compute_replacement, validate as validate_continuous, spline, eligible
OUT=ROOT/'.setup/moving-upper-trial'

def compute(config,q,obj,v,advance,geometry):
 if config.get('tracking_grasp_policy',{}).get('reusable_intercept_path'):
  from reusable_intercept import load_and_admit
  return load_and_admit(config,q,obj,v,advance,geometry)
 if config.get('intercept_window'):
  from intercept_window import compute_intercept
  return compute_intercept(config,q,obj,v,advance,geometry)
 (OUT/'latest-compute-input.json').write_text(json.dumps(dict(q=np.asarray(q).tolist(),object_pose=np.asarray(obj).tolist(),velocity=np.asarray(v).tolist(),advance=advance,geometry=geometry,time=time.time())))
 r=planning_robot(config);obj=forecast(obj,v,advance);initial=r.ee_fk('right',q);g=np.array(config['grasp_object_from_right_ee']);yaw=float(np.degrees(np.arctan2(obj[1,0],obj[0,0]))%180);g[:3,:3]=Rotation.from_euler('z',np.clip(yaw,30,150)-yaw,degrees=True).as_matrix()@g[:3,:3]
 candidates=[]
 for flip in [0,180]:
  grasp=g.copy();grasp[:3,:3]=Rotation.from_euler('z',flip,degrees=True).as_matrix()@g[:3,:3];angle=float(Rotation.from_matrix((obj@grasp)[:3,:3]@initial[:3,:3].T).magnitude());candidates.append((angle,flip,grasp))
 candidates.sort(key=lambda x:x[0]);reasons=[]
 for approach in [.8,1.2,1.6]:
  for angle,flip,grasp in candidates:
   if not roi_contains(geometry,[forecast(obj,v,t)[:3,3] for t in np.linspace(0,approach+.6+(0 if config.get('progressive_descent') else 1),15)]):reasons.append('future target leaves ROI');continue
   align=max(.6,1.875*angle/np.radians(30));result=integrate(r,config,q,obj,v,grasp,approach,align,tracking_window_s=.6,residual_warning_only=True)
   if not result['valid']:reasons.append(dict(reason=result['reason'],approach_s=approach,flip=flip,position_error_m=result['max_position_error_m'],rotation_error_rad=result['max_rotation_error_rad'],failed_at_s=result['times'][-1]));continue
   ts=np.array(result['times']);path=np.array(result['path']);end=path[-1];qd=np.array(result['terminal_velocity']);brake=max(.6,float(np.max(abs(qd)/(.7*r.acceleration))))
   bt=np.linspace(0,brake,int(np.ceil(brake/.02))+1)[1:];bp=end+qd*(bt-bt*bt/(2*brake))[:,None]
   # Braking is part of the reviewed path, and cannot be omitted after approval.
   brake_start=float(ts[-1]);velocities=np.vstack([result['velocities'],qd*(1-bt/brake)[:,None]])
   ts=np.r_[ts,ts[-1]+bt];path=np.vstack([path,bp])
   try:
    for row in path:
     r.clearance(row,config['obstacles'],config['collision_margin'])
     if clearance(r,row)<0:raise ValueError('conveyor envelope during braking')
    if not roi_contains(geometry,[forecast(obj,v,t)[:3,3] for t in np.linspace(0,float(ts[-1])+(0 if config.get('progressive_descent') else 1),20)]):raise ValueError('ROI exit before braking finishes')
   except ValueError as e:reasons.append(str(e));continue
   validated=dict(times=ts.tolist(),path=path.tolist(),velocities=velocities.tolist())
   try:validate_continuous(config,validated,r)
   except ValueError as e:reasons.append(str(e));continue
   return dict(cartesian_residual=dict(policy='warning_only',exceeded=result['residual_exceeded'],position_m=result['max_position_error_m'],rotation_rad=result['max_rotation_error_rad']),times=ts.tolist(),path=path.tolist(),velocities=velocities.tolist(),initial_pose=initial.tolist(),grasp=grasp.tolist(),brake_start_s=brake_start,execution_end_s=8. if config.get('progressive_descent') else 4.,approach_s=approach,tracking_s=.6,braking_s=brake,flip=flip,object_at_start=obj.tolist(),velocity=v.tolist(),geometry=geometry)
 (OUT/'latest-compute-failure.json').write_text(json.dumps(dict(time=time.time(),reasons=reasons),indent=2))
 raise ValueError('no reviewed moving upper path: '+str(reasons[-4:]))

class MovingTrial(Trial):
 def __init__(self):
  super().__init__();self.async_replan=os.environ.get('DEUX_MOVING_UPPER_ASYNC')=='1';self.auto_execute=os.environ.get('DEUX_MOVING_UPPER_AUTO')=='1';self.plan_lead=1.8;self.planning_target=None;self.objects=None;self.track={};self.track_received=0.;self.calibration={};self.cal_received=0.;self.target_id='';self.pool=ProcessPoolExecutor(1,mp_context=multiprocessing.get_context("spawn"));self.camera=None
  from sensor_msgs.msg import CameraInfo
  self.create_subscription(CameraInfo,'/zed/color/camera_info',lambda m:setattr(self,'camera',m),qos_profile_sensor_data)
  self.create_subscription(ObjectDetectionArray,'/task/prediction_detections',self.receive_prediction,qos_profile_sensor_data)
  self.create_subscription(String,'/task/icp_preview_status',self.track_status,10)
  self.create_subscription(String,'/task/camera_calibration_status',self.cal_status,10)
  self.seen_targets=set();self.repeat_deadline=None;self.validation_samples=[];self.repeat_results=[];self.last_planning_failure='';self.reason=('비동기 상부 접근 실행기 대기 · 하강/손 명령 없음' if self.async_replan else 'Moving upper approach only; waiting for GUI request');OUT.mkdir(exist_ok=True)
 def receive_prediction(self,m):
  # Update the causal cache while the planning worker is running, not only when
  # the execution loop next asks for a target. Never refresh acquisition time.
  previous=self.objects
  if previous is not None and seconds(m.header.stamp)<seconds(previous.header.stamp):return
  self.objects=m
  held=getattr(self,'observation_held',False)
  try:MovingTrial.tracked_object(self,allow_hold=False)
  except ValueError:pass  # Invalid samples cannot replace the last valid estimate.
  finally:self.observation_held=held
 def track_status(self,m):self.track=json.loads(m.data);self.track_received=time.monotonic()
 def cal_status(self,m):self.calibration=json.loads(m.data);self.cal_received=time.monotonic()
 def status(self):
  m=TaskStatus(state=self.state,reason=self.reason,active=self.action.busy,workflow='MovingUpperTrial',target_id=self.target_id);m.header.stamp=ros_time(self.now());self.pub.publish(m)
 def tracked_object(self,allow_hold=False):
  if not self.calibration.get('valid') or time.monotonic()-self.cal_received>2.:raise ValueError('camera calibration unavailable')
  if not self.track.get('tracking') or time.monotonic()-self.track_received>1.:raise ValueError('waiting for ROI entry')
  now=self.now();m=self.objects;tid=self.track.get('target_id')
  self.observation_held=False
  cached=getattr(self,'_last_valid_prediction',None)
  if cached is not None and cached[0]!=tid:self._last_valid_prediction=None;cached=None
  error='fresh ICP prediction unavailable'
  if m is not None and 0<=now-seconds(m.header.stamp)<=.5:
   found=[d for d in m.detections if d.track_id==tid and d.velocity_valid]
   error='stable tracked object unavailable'
   if len(found)==1:
    d=found[0];v=np.array([d.velocity.linear.x,d.velocity.linear.y,d.velocity.linear.z,d.velocity.angular.z]);obj=pose_matrix(d.pose.pose)
    if not np.isfinite(v).all() or not np.isfinite(obj).all() or np.linalg.norm(v[:3])>.2 or abs(v[3])>1e-8:
     self._last_valid_prediction=None
     raise ValueError('trial velocity outside straight-belt envelope')
    stamp=seconds(m.header.stamp)
    if cached is None or stamp>=cached[3]:
     self._last_valid_prediction=(tid,obj.copy(),v.copy(),stamp)
     return tid,obj,v,stamp
  limit=float(self.config.get('tracking_grasp_policy',{}).get('prediction_hold_s',.5))
  if allow_hold and cached is not None and 0<=now-cached[3]<=limit:
   self.observation_held=True
   # Original acquisition timestamp is retained: polling cannot renew the lease.
   return cached[0],cached[1].copy(),cached[2].copy(),cached[3]
  raise ValueError(error)
 def revalidate(self,planned,goal,executing=False):
  actual=self.gate(goal);tid,obj,v,stamp=self.tracked_object(allow_hold=executing or bool(self.config.get("tracking_grasp_policy",{}).get("entry_prediction_s")))
  if tid!=planned['target_id']:raise ValueError('tracked target changed')
  policy=self.config.get('tracking_grasp_policy',{})
  if not executing and policy.get('entry_prediction_s') and not planned.get('fixed_interception'):
   from deux_task_core.tracking_policy import entry_prediction
   inside,point=entry_prediction(policy,obj,v,stamp,self.now())
   event=dict(time=self.now(),target_id=tid,stage='dispatch_entry_gate',inside=inside,predicted_position=point.tolist(),observation_age_s=self.now()-stamp)
   with (OUT/'entry-prediction.jsonl').open('a') as f:f.write(json.dumps(event)+'\n')
   if not inside:raise ValueError('one-second predicted object outside XY workspace')
  # Compare predicted poses at the same timestamp, not raw delayed positions.
  now=self.now();live=forecast(obj,v,now-stamp);expected=forecast(np.array(planned['source_object']),np.array(planned['velocity']),now-planned['source_stamp'])
  pe=float(np.linalg.norm(live[:3,3]-expected[:3,3]));ae=float(Rotation.from_matrix(live[:3,:3]@expected[:3,:3].T).magnitude());ve=float(np.linalg.norm(v[:3]-np.array(planned['velocity'])[:3]))
  event=dict(time=now,elapsed_s=now-planned['start_at'],executing=executing,position_error_m=pe,angle_error_deg=float(np.degrees(ae)),velocity_error_m_s=ve,expected_position=expected[:3,3].tolist(),observed_position=live[:3,3].tolist(),observation_age_s=now-stamp)
  if pe>.03 or ae>np.radians(10) or ve>.03 or not self.validation_samples or now-self.validation_samples[-1]['time']>.1:self.validation_samples.append(event)
  if pe>.03:raise ValueError('target prediction changed >3cm')
  if ae>np.radians(10):raise ValueError('target orientation changed >10deg')
  if ve>.03:raise ValueError('belt velocity changed >3cm/s')
  if not executing and max(abs(actual-np.array(planned['start_arm'])))>.03:raise ValueError('arm moved after preview')
  return actual
 async def fresh_gate(self,goal):
  while True:
   try:return self.gate(goal)
   except RuntimeError as e:
    if str(e)!='arm feedback stale':raise
    self.state='WaitingFeedback';self.reason='최신 관절 피드백 대기; 명령 없음';await pause(self,.05)
 async def execute_once(self,goal):
  handle=None;pending=None;result=MoveToReady.Result();report={};completed=False;self.validation_samples=[]
  try:
   while True:
    if self.repeat_deadline is not None and time.monotonic()>self.repeat_deadline:raise TimeoutError('repeat observation window ended')
    await self.fresh_gate(goal);self.state='WaitingTarget';self.reason='이동 상부 접근: 안정된 ROI 대상 대기'+(' · 최근 계획 실패: '+self.last_planning_failure if self.last_planning_failure else '')
    try:tid,obj,v,stamp=self.tracked_object()
    except ValueError:
     await pause(self,.1);continue
    if (tid,stamp)==getattr(self,'last_reuse_observation',None):await pause(self,.03);continue
    if self.config.get('tracking_grasp_policy',{}).get('reusable_intercept_path'):self.last_reuse_observation=(tid,stamp)
    if tid in self.seen_targets:self.reason='다음 ROI 진입 봉투 대기';await pause(self,.1);continue
    policy=self.config.get('tracking_grasp_policy',{})
    if self.camera is None:await pause(self,.1);continue
    start=await self.fresh_gate(goal)
    if len(self.arm.velocity)!=14 or max(abs(np.array(self.arm.velocity)))>.05:await pause(self,.1);continue
    q=np.array(self.config['home_joints']);q[self.idx]=start;q[:7]=self.config.get('_stationary_left_arm',q[:7]);self.target_id=tid
    geometry=dict(camera=json.loads((ROOT/'calibration/zed2i_extrinsics.thor.json').read_text())['T_base_camera_optical'],projection=list(self.camera.p),width=self.camera.width,height=self.camera.height,roi=json.loads((ROOT/'.setup/single-track-roi.json').read_text()))
    if self.planning_target!=tid:self.plan_lead=1.8;self.planning_target=tid
    self.last_planning_failure=''
    compute_config=dict(self.config,_tracking_target_id=tid,_seed_config_version=self.config_version)
    compute_began=self.now();start_at=compute_began+(0. if self.auto_execute and policy.get('fixed_interception') else (self.plan_lead if self.auto_execute else 3.5));job=self.pool.submit(compute,compute_config,q,obj,v,start_at-stamp,geometry)
    (OUT/'latest-intercept-input.json').write_text(json.dumps(dict(config=self.config,q=q.tolist(),object_pose=obj.tolist(),velocity=v.tolist(),advance=start_at-stamp,geometry=geometry,target_id=tid,stamp=stamp,start_at=start_at)))
    self.state='Planning';self.reason=('저장된 고정 경로 · 봉투 통과 위치·도착 시각 확인' if policy.get('reusable_intercept_path') else '작업공간 고정점 후보 · 접근·하강·닫힘 경로 사전 검증' if policy.get('fixed_interception') else '진입 전 1초 선행 접근 경로 준비' if policy.get('preplan_entry') else '작업영역 기반 X·회전 추종 및 XYZ 접근 경로 검증')
    while not job.done():await self.fresh_gate(goal);await pause(self,.02)
    try:planned=job.result()
    except (ValueError,RuntimeError) as e:
     self.last_planning_failure=str(e);self.reason=str(e)
     with (OUT/'intercept-timing.jsonl').open('a') as f:f.write(json.dumps(dict(target_id=tid,compute_s=self.now()-compute_began,lead_s=start_at-compute_began,source_age_s=compute_began-stamp,error=str(e)))+'\n')
     await pause(self,.1);continue
    with (OUT/'intercept-timing.jsonl').open('a') as f:f.write(json.dumps(dict(target_id=tid,compute_s=self.now()-compute_began,lead_s=start_at-compute_began,source_age_s=compute_began-stamp,remaining_s=start_at-self.now(),valid=True))+'\n')
    if self.auto_execute and planned.get('fixed_interception'):
     # The verified upper path is spatially fixed. Assign its execution epoch
     # after computing, instead of racing a deadline guessed before computing.
     from fixed_interception import rebase
     try:
      live_tid,obj,v,stamp=self.tracked_object(allow_hold=True)
      if live_tid!=tid:raise ValueError('selected target changed before dispatch')
      start_at=self.now()+.30
      planned=rebase(planned,obj,v,stamp,start_at,policy,self.config['approach_height'])
     except ValueError as e:
      self.last_planning_failure=str(e);await pause(self,.05);continue
    elif self.auto_execute and policy.get('preplan_entry'):
     template=planned
     from preplanned_entry import correct
     from deux_task_core.tracking_policy import entry_prediction
     self.state='WaitingEntry';self.reason='사전 경로 준비 완료 · 1초 뒤 작업공간 진입 대기'
     try:
      while True:
       actual=await self.fresh_gate(goal)
       latest_tid,obj,v,stamp=self.tracked_object(allow_hold=True)
       if latest_tid!=tid:raise ValueError('preplanned target changed')
       q[self.idx]=actual;start=actual
       correction_began=self.now()
       budget=max(.25,min(1.,getattr(self,'correction_budget_s',.52)))
       start_at=correction_began+budget+.20
       job=self.pool.submit(correct,compute_config,template,q,obj,v,start_at-stamp,geometry)
       self.state='PreparingEntry';self.reason='진입 전 최신 경로를 계속 보정 중'
       while not job.done():await self.fresh_gate(goal);await pause(self,.01)
       planned=job.result();duration=self.now()-correction_began
       self.correction_budget_s=duration+.03
       live_tid,live_obj,live_v,live_stamp=self.tracked_object(allow_hold=True)
       if live_tid!=tid:raise ValueError('preplanned target changed')
       inside,point=entry_prediction(policy,live_obj,live_v,live_stamp,self.now())
       with (OUT/'entry-prediction.jsonl').open('a') as f:f.write(json.dumps(dict(time=self.now(),target_id=tid,stage='rolling_prepared',inside=inside,compute_s=duration,remaining_s=start_at-self.now(),trigger_position=point.tolist(),start_at=start_at))+'\n')
       if (inside or planned.get('fixed_interception')) and start_at-self.now()>=.15:break
       await pause(self,.01)
     except ValueError as e:
      self.last_planning_failure=str(e)
      with (OUT/'entry-prediction.jsonl').open('a') as f:f.write(json.dumps(dict(time=self.now(),target_id=tid,stage='preplanned_correction_rejected',reason=str(e)))+'\n')
      await pause(self,.05);continue
    if self.now()>start_at-(.15 if self.auto_execute and policy.get("preplan_entry") else (.65 if self.auto_execute else 1.3)):
     if self.auto_execute:self.plan_lead=min(3.5,self.now()-compute_began+.8)
     self.reason='계산 중 시작 시간 여유 부족; 재계산';await pause(self,.1);continue
    planned.update(target_id=tid,source_object=obj.tolist(),source_stamp=stamp,start_arm=start.tolist(),start_at=start_at)
    if planned.get('intercept'):self.intercept_plan=dict(planned['intercept'],target_id=tid,arrival_at=start_at+planned.get('fixed_interception',{}).get('nominal_contact_s',planned['approach_s']))
    t=np.array(planned['times']);q28=np.array(planned['path']);qa=q28[:,self.idx]
    from deux_task_core.model import ARM_NAMES
    tr=JointTrajectory(joint_names=list(ARM_NAMES),points=[JointTrajectoryPoint(positions=row.tolist(),time_from_start=ros_time(float(tm),Duration)) for tm,row in zip(t,qa)]);tr.header.stamp=ros_time(start_at)
    if self.async_replan:
     for point,velocity in zip(tr.points,planned['velocities']):point.velocities=np.array(velocity)[self.idx].tolist()
     # Full cubic validation completed in worker before returning this plan.
     pass
    else:validate_limits(tr,ARM_NAMES,self.robot)
    plan=MotionPlan(plan_id=uuid.uuid4().hex,stage='MovingUpperApproachOnly',follows_conveyor=True,scene_version=self.config_version,trajectory=tr)
    if self.auto_execute:
     await pause(self,.01)
     try:self.revalidate(planned,goal)
     except (ValueError,RuntimeError) as e:
      self.reason=str(e)
      with (OUT/'entry-prediction.jsonl').open('a') as f:f.write(json.dumps(dict(time=self.now(),target_id=tid,stage='dispatch_rejected',reason=str(e)))+'\n')
      await pause(self,.05);continue
     if self.now()>start_at-(.10 if self.auto_execute and policy.get("preplan_entry") else .5):continue
     report['approval']='user-authorized one-shot automatic upper trial'
    else:
     self.review.data=preview(self.robot,plan,q28[0],t,q28)
     deadline=start_at-self.now()-.6;self.review.ticket=Ticket(plan.plan_id,self.review.data['digest'],deadline)
     self.review.data.update(approvable=True,request_id=self.review.ticket.request_id,warning=f'Moving upper trial ONLY. Scheduled ROS start {start_at:.3f}; approve before countdown expires. Approach {planned["approach_s"]:.1f}s + track 0.6s + brake {planned["braking_s"]:.1f}s. No descent or hand command. Prediction/ROI loss cancels. Provisional conveyor geometry.')
     self.state='AwaitingApproval';self.reason='이동 상부 접근 1회 · 남은 시간 안에 GUI 승인 · 만료 시 재계산'
     approved=False
     await pause(self,.05)
     try:
      while self.review.ticket.decision is None and time.monotonic()<self.review.ticket.deadline:
       self.revalidate(planned,goal);self.review.publish('pending',self.reason);await pause(self,.05)
      if self.review.ticket.decision=='reject':raise InterruptedError('operator rejected')
      approved=self.review.ticket.decision=='approve'
     except (ValueError,RuntimeError) as e:self.reason=str(e)
     if not approved:
      self.review.publish('closed',self.reason);self.review.ticket=None;self.review.data=None;await pause(self,.05);continue
    self.revalidate(planned,goal)
    if self.now()>start_at-(.10 if self.auto_execute and policy.get("preplan_entry") else .5):raise ValueError('approval too late; no command sent')
    self.review.publish('approved');self.used=True;report=planned;report['auto_execute']=self.auto_execute;self.state='Executing';self.reason='상부 접근·짧은 추종·정지 1회 실행 중'
    pending=self.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=tr));handle=await self.future(pending,goal,seconds=.4)
    if not handle.accepted:raise RuntimeError('controller rejected scheduled trajectory')
    accepted_curve=spline(planned) if self.async_replan else None
    done=handle.get_result_async();deadline=time.monotonic()+max(0.,start_at-self.now())+float(planned['execution_end_s'])+3.
    job=None;generation=0;replacement_count=0;replan_lead=.7;compute_history=[];last_stamp=stamp;replan_events=[];stop_replanning=False;last_replan_at=-float("inf");hold_logged=False
    while not done.done():
     elapsed=self.now()-start_at
     if self.async_replan:
      actual=self.gate(goal)
      # Hardware and tracking guards remain active. Perception loss leaves the checked braking tail.
      observation_ok=True
      try:
       latest_tid,latest_obj,latest_v,latest_stamp=self.tracked_object(allow_hold=True)
       if latest_tid!=tid:raise ValueError('tracked target changed')
      except ValueError as e:
       observation_ok=False;latest_stamp=last_stamp
       if not replan_events or replan_events[-1].get('reason')!=str(e):replan_events.append(dict(time=self.now(),reason=str(e)))
       if str(e) not in ('fresh ICP prediction unavailable','stable tracked object unavailable'):stop_replanning=True
      held=observation_ok and getattr(self,'observation_held',False)
      if held!=hold_logged:
       replan_events.append(dict(time=self.now(),prediction_hold=held,observation_age_s=self.now()-latest_stamp))
       hold_logged=held
      if hasattr(self,'progressive_tick'):
       await self.progressive_tick(goal,planned,actual,elapsed,observation_ok,
                                   (latest_obj,latest_v,latest_stamp) if observation_ok else None)
       if self.progressive_closed:stop_replanning=True
       if getattr(self,'abort_descent',False):stop_replanning=True
      if job is not None and job.done():
       began,splice,submitted_stamp=job_meta
       try:
        replacement=job.result()
        compute_history.append(self.now()-began);replan_lead=min(1.2,max(.4,max(compute_history[-8:])+.25))
        if stop_replanning or not observation_ok or not eligible(replacement,self.now(),tid,generation):raise ValueError('obsolete/late replacement')
        # Recheck current observation against the one used by the worker before dispatch.
        a=forecast(latest_obj,latest_v,self.now()-latest_stamp)
        b=forecast(np.array(replacement['source_object']),np.array(replacement['velocity']),self.now()-replacement['source_stamp'])
        if np.linalg.norm(a[:3,3]-b[:3,3])>.03 or Rotation.from_matrix(a[:3,:3]@b[:3,:3].T).magnitude()>np.radians(10):raise ValueError('replacement observation superseded')
        if np.linalg.norm(latest_v[:3]-np.array(replacement['velocity'])[:3])>.03:raise ValueError('replacement velocity superseded')
        new_tr=JointTrajectory(joint_names=list(ARM_NAMES),points=[JointTrajectoryPoint(positions=np.array(row)[self.idx].tolist(),velocities=np.array(vel)[self.idx].tolist(),time_from_start=ros_time(float(tm),Duration)) for tm,row,vel in zip(replacement['times'],replacement['path'],replacement['velocities'])])
        new_tr.header.stamp=ros_time(start_at)
        # Same original header and identical prefix: do not cancel old action first.
        pending=self.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=new_tr))
        new_handle=await self.future(pending,goal,seconds=.1)
        if not new_handle.accepted:raise RuntimeError('controller rejected replacement')
        handle=new_handle;done=handle.get_result_async();planned=replacement;accepted_curve=spline(planned)
        t=np.array(planned['times']);qa=np.array(planned['path'])[:,self.idx]
        replacement_count+=1;report=planned
        replan_events.append(dict(time=self.now(),accepted=True,splice_s=splice,compute_s=self.now()-began,observation_stamp=submitted_stamp,updated_window_s=replacement['updated_window_s'],planning_timing_ms=replacement['planning_timing_ms']))
       except ValueError as e:replan_events.append(dict(time=self.now(),accepted=False,reason=str(e),compute_s=self.now()-began,splice_s=splice,remaining_to_splice_s=start_at+splice-self.now()))
       job=None
      if (job is None and observation_ok and not stop_replanning and (latest_stamp>last_stamp or (held and self.now()-last_replan_at>=.2))
          and elapsed+replan_lead<min(planned['brake_start_s']-.05,planned['execution_end_s']-.2)):
       generation+=1;splice=max(.25,elapsed+replan_lead)
       job_config=dict(self.config)
       if hasattr(self,'descent_admitted'):
        job_config['_descent_admitted']=self.descent_admitted
        job_config['_descent_measured_height']=getattr(self,'descent_measured_height',self.config['approach_height'])
        job_config['_descent_delay_s']=getattr(self,'descent_delay_s',0.)
       job=self.pool.submit(compute_replacement,job_config,planned,latest_obj,latest_v,latest_stamp,splice,generation)
       job_meta=(self.now(),splice,latest_stamp);last_stamp=latest_stamp;last_replan_at=self.now()
      expected=accepted_curve(np.clip(elapsed,0,t[-1]))[self.idx]
      report['async_replan']=dict(replacements=replacement_count,events=replan_events)
     else:
      actual=self.revalidate(planned,goal,executing=True);expected=sample(t,qa,max(0.,elapsed))
     if max(abs(actual-expected))>self.config['tracking_tolerance']:raise RuntimeError('tracking error in moving trial')
     if time.monotonic()>deadline:raise TimeoutError('moving trial completion timeout')
     await pause(self,.02)
    response=done.result()
    if response.status!=4 or response.result.error_code:raise RuntimeError('arm action failed '+response.result.error_string)
    error=self.gate(goal)-qa[-1];report['final_error_deg']=np.degrees(error).tolist()
    if max(abs(error))>self.config['final_tolerance']:raise ValueError('final upper holding error')
    completed=True;handle=None;pending=None;result.success=True;result.reason=('Tracking motion and braking completed; see progressive_descent closure report' if hasattr(self,'progressive_tick') else 'Moving upper approach, short tracking and braking completed; no descent/hand command');self.state='Ready';break
  except Exception as e:result.success=False;result.reason=f'{type(e).__name__}: {e}';self.state='Stopped'
  finally:
   if pending is not None and not completed:
    def late(f):
     try:
      h=f.result()
      if h.accepted:h.cancel_goal_async()
     except Exception:pass
    pending.add_done_callback(late)
   if not completed:await cancel_children(self,[handle])
   terminal_confirmed=completed or pending is None
   if handle is not None and handle.accepted:
    terminal=handle.get_result_async();end=time.monotonic()+3.
    while not terminal.done() and time.monotonic()<end:await pause(self,.02)
    terminal_confirmed=terminal.done()
    if terminal.done():report['controller_terminal_status']=terminal.result().status
   elif pending is not None:terminal_confirmed=pending.done() and not pending.result().accepted
   if pending is not None and not completed:
    # A replacement may be accepted after its response timeout. Its termination
    # must be confirmed independently of the preempted old goal.
    end=time.monotonic()+3.
    while not pending.done() and time.monotonic()<end:await pause(self,.02)
    if not pending.done():terminal_confirmed=False
    elif pending.result().accepted:
     pending_handle=pending.result();pending_terminal=pending_handle.get_result_async()
     while not pending_terminal.done() and time.monotonic()<end:await pause(self,.02)
     terminal_confirmed=terminal_confirmed and pending_terminal.done()
   report['controller_terminal_confirmed']=terminal_confirmed
   report['validation_samples']=self.validation_samples
   if hasattr(self,'progressive_events'):
    report['progressive_descent']=dict(events=self.progressive_events,right_close_sent=bool(self.close_commands),closure_completed=self.progressive_closed,hand_commands=self.close_commands)
   if self.review.ticket is not None:self.review.publish('closed',result.reason)
   self.review.ticket=None;self.review.data=None;self.reason=result.reason;report.update(success=result.success,reason=result.reason);(OUT/'result.json').write_text(json.dumps(report,indent=2))
  self.last_report=report
  return result
 async def recover_ready(self,goal):
  if not hasattr(self,'right_trial_hand'):
   from right_trial_hand import RightTrialHand
   self.right_trial_hand=RightTrialHand(self)
  opened=await self.right_trial_hand.open_ready(goal)
  (OUT/'ready-right-hand.json').write_text(json.dumps(opened,indent=2))
  actual=await self.fresh_gate(goal);ready=np.array(self.config['home_joints']);self.target=ready[14:21].copy()
  if max(abs(actual[7:]-ready[14:21]))<=self.config['final_tolerance']:return MoveToReady.Result(success=True,reason='ready within tolerance')
  q=np.array(self.config['home_joints']);q[self.idx]=actual;q[:7]=self.config.get('_stationary_left_arm',q[:7])
  from ready_recovery import compute_ready
  self.state='ReadyRecoveryPlanning';self.reason='반복 시험: 현재 관절에서 준비자세 복귀 경로 계산'
  job=self.pool.submit(compute_ready,self.config,q.tolist(),ready.tolist())
  while not job.done():self.gate(goal);await pause(self,.03)
  path=job.result();(ROOT/'.setup/ready-minus30-path/path.json').write_text(json.dumps(path,indent=2))
  # The user explicitly requested no GUI approvals for this repeated trial.
  if os.environ.get('DEUX_READY_AUTO')!='1':raise RuntimeError('automatic ready recovery not enabled')
  return await Trial.execute(self,goal)
 async def execute(self,goal):
  if os.environ.get('DEUX_MOVING_UPPER_REPEAT')!='1':return await self.execute_once(goal)
  self.repeat_deadline=time.monotonic()+300.;self.repeat_results=[];self.seen_targets=set()
  history=OUT/'repeat-results.json'
  if history.exists():(OUT/f'repeat-results-{time.time_ns()}.json').write_text(history.read_text())
  history.write_text('[]')
  allowed=('ValueError: target prediction changed','ValueError: target orientation changed','ValueError: belt velocity changed','ValueError: tracked target changed','ValueError: waiting for ROI entry','ValueError: fresh ICP prediction unavailable','ValueError: stable tracked object unavailable')
  for index in range(5):
   self.used=False
   recovery=await self.recover_ready(goal)
   if not recovery.success:return MoveToReady.Result(success=False,reason='repeat halted during ready recovery: '+recovery.reason)
   self.used=False
   result=await self.execute_once(goal)
   report=dict(self.last_report,trial=index+1)
   self.repeat_results.append(report)
   (OUT/'repeat-results.json').write_text(json.dumps(self.repeat_results,indent=2))
   if report.get('target_id'):self.seen_targets.add(report['target_id'])
   if goal.is_cancel_requested:return result
   if not report.get('controller_terminal_confirmed',False):return MoveToReady.Result(success=False,reason='repeat halted: controller termination unconfirmed')
   if not result.success and not result.reason.startswith(allowed):return MoveToReady.Result(success=False,reason='repeat halted: '+result.reason)
   if index==4:break
   # Confirm quiet fresh feedback for half a second before allowing another target.
   self.state='Settling';self.reason=f'반복 시험 {index+1}/5 종료 · 정지 확인 후 다음 봉투 대기';quiet=None;end=time.monotonic()+5
   while time.monotonic()<end:
    self.gate(goal)
    if len(self.arm.velocity)==14 and max(abs(np.array(self.arm.velocity)))<.05:
     quiet=quiet or time.monotonic()
     if time.monotonic()-quiet>.5:break
    else:quiet=None
    await pause(self,.05)
   else:return MoveToReady.Result(success=False,reason='repeat halted: arm did not settle')
  success=sum(bool(r.get('success')) for r in self.repeat_results);self.state='Finished';self.reason=f'반복 시험 5회 종료 · 완료 {success}/5';self.used=True
  return MoveToReady.Result(success=success==5,reason=self.reason)

if __name__=='__main__':
 rclpy.init();node=MovingTrial()
 try:rclpy.spin(node)
 finally:node.pool.shutdown(wait=False,cancel_futures=True);node.destroy_node();rclpy.shutdown()
