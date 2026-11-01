from right_only_trial import planning_robot
"""Single fresh-observation descent + right closure trial; no automatic return/open."""
import json,time,copy,types,os
from pathlib import Path
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.action import MoveToReady
from deux_task_core.model import RobotModel,ARM_NAMES
from deux_task_core.conveyor import forecast
from deux_task_core.continuous_grasp_preview import plan
from deux_task_core.live_continuous_preview import roi_contains
from deux_task_runtime.common import pause,ros_time,cancel_children
from moving_upper_trial import MovingTrial,OUT,ROOT
from async_upper import validate,spline
from progressive_descent import CLOSE_POSITION_TOLERANCE_M


def compute_descent(config,q,obj,v,advance,grasp,geometry,descent_s=.5):
 c=copy.deepcopy(config);c['grasp_object_from_right_ee']=grasp;r=planning_robot(c);obj=forecast(np.array(obj),np.array(v),advance);q=np.array(q);v=np.array(v)
 if not roi_contains(geometry,[forecast(obj,v,t)[:3,3] for t in np.linspace(0,descent_s+.5,20)]):raise ValueError('descent/closure forecast leaves ROI')
 result=plan(types.SimpleNamespace(robot=r,config=c),q,np.zeros(28),obj,v,descent_s,.5,residual_warning_only=True,bound_joints=True)
 if not result['valid']:raise ValueError(result['reason'])
 t=np.array(result['times']);path=np.array(result['path']);vel=np.array(result['velocities'])
 final=r.ee_fk('right',path[-1]);target=forecast(obj,v,float(t[-1]))@np.array(grasp)
 error=float(np.linalg.norm(final[:3,3]-target[:3,3]));angle=float(Rotation.from_matrix(target[:3,:3]@final[:3,:3].T).magnitude())
 # Closure is a distinct eligibility check, not the relaxed upper-approach gate.
 if error>CLOSE_POSITION_TOLERANCE_M:raise ValueError(f'closure alignment insufficient: {error:.4f}m {angle:.4f}rad')
 braking=max(.6,float(np.max(abs(vel[-1])/(.7*r.acceleration))));bt=np.linspace(0,braking,int(np.ceil(braking/.02))+1)[1:]
 result.update(times=np.r_[t,t[-1]+bt].tolist(),path=np.vstack([path,path[-1]+vel[-1]*(bt-bt*bt/(2*braking))[:,None]]).tolist(),velocities=np.vstack([vel,vel[-1]*(1-bt/braking)[:,None]]).tolist(),object_at_start=obj.tolist(),velocity=v.tolist(),descent_s=descent_s,close_s=.5,closure_position_error_m=error,closure_rotation_error_rad=angle)
 result['minimum_clearance_m']=validate(c,result,r)
 return result


class GraspTrial(MovingTrial):
 def __init__(self):
  super().__init__()
  self.async_replan=True  # Workspace policy requires continuous replacements.
  from right_only_trial import RightOnlyClient
  self.client=RightOnlyClient(self.client)
  self.passive_left=None
  self.descent_admitted=False;self.progressive_closed=False;self.close_started=None;self.close_commands=[];self.progressive_events=[]
  self.intercept_zone=json.loads((ROOT/'calibration/intercept_window.json').read_text())
  from deux_task_core.tracking_policy import validate_policy
  self.config['tracking_grasp_policy']=validate_policy(json.loads((ROOT/'calibration/tracking_grasp_policy.json').read_text()))
  policy=self.config['tracking_grasp_policy']
  if policy.get('standby_right_arm_rad'):
   self.config['home_joints']=list(self.config['home_joints'])
   self.config['home_joints'][14:21]=policy['standby_right_arm_rad']
  self.config['approach_height']=float(policy.get('approach_height_m',self.config['approach_height']))
  self.config['right_hand_ready_fraction']=policy['ready_fraction']
  self.config['upper_replan_horizon_s']=.8
  self.pregrasp_started=None
  # Query/cache profiles while idle; no grip command is sent here.
  from right_trial_hand import RightTrialHand
  self.right_trial_hand=RightTrialHand(self)
  from deux_task_core.planning import scene_version
  self.config_version=scene_version(self.config)

  from intercept_window import validate_zone
  validate_zone(self.intercept_zone)
  from std_msgs.msg import String
  self.intercept_pub=self.create_publisher(String,'/task/intercept_window',10)
  self.create_timer(.25,self.publish_intercept_window)
  self.reason='시간 기준 통일 · 파지 구간 내 전체 경로 검증 · 실행 대기'
 def admit(self,request):
  try:return super().admit(request)
  except Exception as error:
   self.state='AdmissionRejected';self.reason=f'시작 조건 미충족: {error}'
   self.status()
   raise

 def publish_intercept_window(self):
  from std_msgs.msg import String
  from intercept_window import polygon
  data=dict(frame='base_link',zone=self.intercept_zone,polygon=polygon(self.intercept_zone),active=self.action.busy,
            tracking_lookahead_s=0.,reason=self.reason,validated=False)
  plan=getattr(self,'intercept_plan',None)
  if plan is not None and self.action.busy and self.now()<=plan['arrival_at']:
   data.update(validated=True,encounter_position=plan['object_position'],arrival_in_s=plan['arrival_at']-self.now(),target_id=plan['target_id'])
  self.intercept_pub.publish(String(data=json.dumps(data)))

 def gate(self,goal=None):
  if goal is not None and goal.is_cancel_requested:raise InterruptedError('GUI canceled')
  if self.estop:raise RuntimeError('e-stop active')
  self.hardware_gate();self.check_model_version()
  if time.monotonic()-self.external<.5:raise RuntimeError('competing arm trajectory command')
  actual=self.measured()
  bounds=self.robot.limits[14:21]
  if np.any(actual[7:]<bounds[:,0]-self.robot.joint_limit_tolerance) or np.any(actual[7:]>bounds[:,1]+self.robot.joint_limit_tolerance):
   violations=[dict(joint=ARM_NAMES[7+i],measured_deg=float(np.degrees(actual[7+i])),limits_deg=np.degrees(bounds[i]).tolist()) for i in range(7) if actual[7+i]<bounds[i,0]-self.robot.joint_limit_tolerance or actual[7+i]>bounds[i,1]+self.robot.joint_limit_tolerance]
   (OUT/'right-limit-event.json').write_text(json.dumps(dict(time=self.now(),violations=violations),indent=2))
   raise RuntimeError('right measured joint limit: '+json.dumps(violations))
  # Right-only trial: left-arm drift is not an execution admission condition.
  if self.passive_left is not None:
   actual=actual.copy();actual[:7]=self.passive_left
  return actual

 async def progressive_tick(self,goal,planned,actual,elapsed,observation_ok,observation):
  if self.config.get('tracking_grasp_policy'):
   return await self.policy_tick(goal,planned,actual,elapsed,observation_ok,observation)
  from progressive_descent import position_errors,close_eligible
  self.descent_admitted=False
  if self.progressive_closed:return
  if not observation_ok:
   if self.close_started is not None:raise ValueError('observation lost during hand closure')
   return
  obj,v,stamp=observation
  q=np.array(self.config['home_joints']);q[self.idx]=actual;q[:7]=self.passive_left
  # Match prediction time to acquisition time of joint feedback, not receive time.
  from deux_task_runtime.common import seconds
  at=seconds(self.arm.header.stamp)
  if at<stamp:
   if self.close_started is not None:raise ValueError('joint observation predates object observation during closure')
   return
  xy,distance,fk,target=position_errors(self.robot,q,obj,v,stamp,at,planned['grasp'])
  from scipy.spatial.transform import Rotation
  angle=float(Rotation.from_matrix(target[:3,:3]@fk[:3,:3].T).magnitude())
  from intercept_window import contains
  object_now=forecast(obj,v,max(at,stamp)-stamp)[:3,3]
  in_window=contains(self.intercept_zone,[object_now])
  self.descent_admitted=elapsed>=planned['approach_s'] and xy<=.12 and in_window
  now=self.now()
  close_points=[forecast(obj,v,now-stamp+t)[:3,3] for t in np.linspace(0,.5,8)]
  inside=roi_contains(planned['geometry'],close_points) and contains(self.intercept_zone,close_points)
  if not self.progressive_events or now-self.progressive_events[-1]['time']>=.1:
   self.progressive_events.append(dict(time=now,elapsed_s=elapsed,xy_error_m=xy,position_error_m=distance,rotation_error_rad=angle,descent_admitted=self.descent_admitted,roi_close_window=inside))
  if self.close_started is None:
   if planned.get('descent_profile') is not None:
    self.state='TrackingDescent';self.reason='위치 추적하며 점진적 하강 · 회전 오차는 기록만'
   if close_eligible(distance,elapsed,planned['brake_start_s'],inside,observation_ok):
    self.right_trial_hand.guard()
    self.close_started=now
  if self.close_started is not None:
   # Position remains required while closing. Rotation is deliberately not a gate.
   if distance>CLOSE_POSITION_TOLERANCE_M or not inside:raise ValueError('closure position/ROI condition lost')
   self.state='TrackingClose';self.reason='위치 정렬 확인 · 추적하며 오른손 0.5초 닫힘'
   self.right_trial_hand.send(1.)
   self.close_commands.append(dict(time=now,grasp=1.,position_error_m=distance,rotation_error_rad=angle))
   if now-self.close_started>=.5:
    self.progressive_closed=True
    self.state='GraspBraking';self.reason='오른손 닫힘 명령 완료 · 검증된 감속 경로로 정지'

 async def policy_tick(self,goal,planned,actual,elapsed,observation_ok,observation):
  from progressive_descent import position_errors,height_state
  from deux_task_core.tracking_policy import forecast_inside,closure_ok,follow_admitted,workspace_descent_ready
  from deux_task_runtime.common import seconds
  p=self.config['tracking_grasp_policy'];self.descent_admitted=False;self.abort_descent=False
  if self.progressive_closed:return
  if not observation_ok:
   if self.close_started is not None:raise ValueError('observation lost during hand closure')
   return
  obj,v,stamp=observation;at=seconds(self.arm.header.stamp);now=self.now()
  if at<stamp:
   if self.close_started is not None:raise ValueError('joint observation predates object during closure')
   # No descent update; keep the existing validated motion and brake tail.
   return
  q=np.array(self.config['home_joints']);q[self.idx]=actual;q[:7]=self.passive_left
  xy,distance,fk,target=position_errors(self.robot,q,obj,v,stamp,at,planned['grasp'])
  height=float(fk[2,3]-target[2,3]);grasp=np.array(planned['grasp'])
  follow=follow_admitted(p,obj,v,grasp,now-stamp,previous=planned.get('workspace_follow',False))
  nominal_height,_=height_state(planned.get('descent_profile'),max(0.,elapsed),self.config['approach_height'])
  timing=None
  if p.get('scheduled_descent'):
   from scheduled_descent import readiness
   schedule=readiness(max(0.,height),xy,v,fk[:3,3],target[:3,3],p,now,planned.get('descent_profile'),max(0.,elapsed))
   from descent_timing import FeedbackTiming
   if not hasattr(self,'feedback_timing'):
    self.feedback_timing=FeedbackTiming(ROOT/'.setup/pose-descent-delay.json',self.config_version)
   timing=self.feedback_timing.update(planned,q,at,now,max(0.,height),p)
   finish=max(schedule['remaining_s'],timing['remaining_s'])
  else:
   finish=max(nominal_height,max(height,0.))/p['descent_speed_m_s']+p['close_duration_s']
   schedule=dict(arrival_s=None)
   self.descent_delay_s=0.
  if timing is not None:self.descent_delay_s=timing['delay_s']
  self.descent_measured_height=max(0.,height)

  points=[(forecast(obj,v,now-stamp+t)@grasp)[:3,3] for t in (0.,finish)]
  entire_inside=forecast_inside(p,obj,v,grasp,now-stamp,now-stamp+finish,p['entry_margin_m'])
  # ROI includes the full future interval, independent of the base XY workspace.
  roi_ok=roi_contains(planned['geometry'],[(forecast(obj,v,now-stamp+t)@grasp)[:3,3] for t in np.linspace(0,finish,8)])
  self.abort_descent=planned.get('descent_profile') is not None and not (follow and entire_inside and roi_ok)
  self.descent_admitted=(schedule['due'] and xy<=.12 and follow and entire_inside and roi_ok
                        if p.get('scheduled_descent') else
                        workspace_descent_ready(p,xy,elapsed,planned['approach_s'],follow,entire_inside,roi_ok))
  fixed=planned.get('fixed_interception')
  if fixed is not None:
   from fixed_interception import arrival,close_offset
   from scheduled_descent import profile_for
   # Latest evidence updates arrival, never the selected point. Feedback is
   # compared at its own timestamp; no receive-time position substitution.
   contact,lateral=arrival(fixed,obj,v,grasp,now-stamp)
   eta=contact-(now-stamp)
   upper=np.asarray(fixed['pose'])[:3,3].copy();upper[2]+=self.config['approach_height']
   settled=np.linalg.norm(fk[:3,3]-upper)<=.02
   velocities=dict(zip(self.arm.name,self.arm.velocity))
   settled=settled and all(name in velocities and abs(velocities[name])<.08 for name in ARM_NAMES[7:])
   profile=planned.get('descent_profile')
   if profile is None and settled:
    nominal=profile_for(max(0.,height),0.,p['descent_speed_m_s'],p['max_descent_m'])
    needed=close_offset(nominal,p['close_height_m'])
    self.descent_admitted=bool(settled and lateral<=p['close_xy_m'] and eta>=needed+.4 and eta<=needed+2. and follow and entire_inside and roi_ok)
   elif profile is None:self.descent_admitted=False
   else:self.descent_admitted=bool(follow and entire_inside and roi_ok)
   schedule.update(arrival_s=eta,fixed_point=fixed['pose'],settled=settled,lateral_m=lateral)
   if self.intercept_plan is not None:
    self.intercept_plan.update(arrival_at=now+eta,arrival_in_s=eta,settled=bool(settled),lateral_m=lateral)
   self.fixed_wait_detail=f'봉투 도착 {eta:.2f}초 · 상부 도착 {"완료" if settled else "이동 중"} · 경로 옆오차 {lateral*100:.1f}cm'
  close_inside=forecast_inside(p,obj,v,grasp,now-stamp,now-stamp+p['close_duration_s']) and roi_contains(planned['geometry'],[(forecast(obj,v,now-stamp+t)@grasp)[:3,3] for t in np.linspace(0,p['close_duration_s'],8)])
  if not self.progressive_events or now-self.progressive_events[-1]['time']>=.1:
   self.progressive_events.append(dict(time=now,elapsed_s=elapsed,xy_error_m=xy,height_error_m=height,
    position_error_m=distance,workspace_follow=follow,descent_admitted=self.descent_admitted,
    predicted_close_inside=close_inside,pregrasp_fraction=p['pregrasp_fraction'],
    prediction_hold=getattr(self,'observation_held',False),observation_age_s=now-stamp,remaining_descent_close_s=finish,arrival_s=schedule['arrival_s'],entire_inside=entire_inside,roi_ok=roi_ok,descent_timing=timing,fixed_interception=schedule if fixed is not None else None))
  if self.close_started is None:
   if self.descent_admitted:
    if self.pregrasp_started is None:self.pregrasp_started=now
    self.right_trial_hand.send(p['pregrasp_fraction'])
   if not follow:
    self.state='TrackingOutsideWorkspace';self.reason='작업영역 밖: X·회전 추적, Y·높이 대기'
   elif planned.get('descent_profile') is not None:
    self.state='TrackingDescent';self.reason='작업영역 내 추적·하강, 오른손 0.4'
   else:
    self.state='TrackingAbove';self.reason=(self.fixed_wait_detail if fixed is not None else '작업영역 내 상부 추적 · 하강/닫힘 예측 조건 대기')
   opened=self.pregrasp_started is not None and now-self.pregrasp_started>=p['pregrasp_duration_s']
   contact_due=fixed is None or (planned.get('descent_profile') is not None and elapsed>=planned['descent_profile']['contact_s'])
   if opened and contact_due and self.descent_admitted and closure_ok(p,xy,height,planned['brake_start_s']-elapsed,close_inside,observation_ok):
    self.close_started=now
  if self.close_started is not None:
   # XY/height gates are retained through closing; rotation is not a contact gate.
   if xy>p['close_xy_m'] or not -.005<=height<=p['close_height_m']+.005 or not close_inside:
    raise ValueError('closure XY/height/workspace condition lost')
   self.state='TrackingClose';self.reason='마지막 1.5cm 하강과 오른손 닫힘 중첩 · 컨베이어 추종'
   self.right_trial_hand.send(1.)
   self.close_commands.append(dict(time=now,grasp=1.,xy_error_m=xy,height_error_m=height,prediction_hold=getattr(self,'observation_held',False),observation_age_s=now-stamp))
   if now-self.close_started>=p['close_duration_s']:
    self.progressive_closed=True;self.state='GraspBraking';self.reason='닫힘 명령 완료 · 검증된 감속 정지'

 async def descend_once(self,goal):
  report={};handle=None;pending=None;closed=False;completed=False
  try:
   self.state='WaitingDescentSettle';self.reason='하강 전 팔 정지 피드백 확인'
   quiet=None;end=time.monotonic()+2.
   while True:
    actual=await self.fresh_gate(goal)
    if len(self.arm.velocity)==14 and max(abs(np.array(self.arm.velocity)))<=.05:
     quiet=quiet or time.monotonic()
     if time.monotonic()-quiet>=.1:break
    else:quiet=None
    if time.monotonic()>end:raise ValueError('arm not settled for descent')
    await pause(self,.02)
   tid,obj,v,stamp=self.tracked_object()
   if tid!=self.last_report['target_id']:raise ValueError('original upper target unavailable; no descent')
   self.right_trial_hand.guard()
   if len(self.arm.velocity)!=14 or max(abs(np.array(self.arm.velocity)))>.05:raise ValueError('arm not settled for descent')
   q=np.array(self.config['home_joints']);q[self.idx]=actual;q[:7]=self.passive_left
   start_at=self.now()+1.5;geometry=self.last_report['geometry'];grasp=self.last_report['grasp']
   self.state='DescentPlanning';self.reason='최신 관측으로 하강 0.5초 + 오른손 닫힘 0.5초 계산'
   job=self.pool.submit(compute_descent,self.config,q,obj,v,start_at-stamp,grasp,geometry)
   while not job.done():self.gate(goal);await pause(self,.02)
   report=job.result();report.update(target_id=tid,start_at=start_at,source_object=obj.tolist(),source_stamp=stamp,start_arm=actual.tolist())
   if self.now()>start_at-.4:raise ValueError('descent planning late; no command')
   self.revalidate(report,goal)
   self.right_trial_hand.guard()
   tr=JointTrajectory(joint_names=list(ARM_NAMES),points=[JointTrajectoryPoint(positions=np.array(row)[self.idx].tolist(),velocities=np.array(vel)[self.idx].tolist(),time_from_start=ros_time(float(t),Duration)) for t,row,vel in zip(report['times'],report['path'],report['velocities'])]);tr.header.stamp=ros_time(start_at)
   self.state='Descending';self.reason='하강 0.5초 → 추종하며 오른손 닫힘 0.5초 → 감속 정지'
   pending=self.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=tr));handle=await self.future(pending,goal,seconds=.25)
   if not handle.accepted:raise RuntimeError('descent controller rejected')
   done=handle.get_result_async();curve=spline(report);events=[]
   while not done.done():
    actual=self.gate(goal);elapsed=self.now()-start_at;desired=curve(np.clip(elapsed,0,report['times'][-1]))[self.idx]
    if max(abs(actual-desired))>self.config['tracking_tolerance']:raise ValueError('descent joint tracking error')
    if elapsed<report['descent_s']+.5:self.revalidate(report,goal,executing=True)
    if report['descent_s']<=elapsed<report['descent_s']+.5:
     self.right_trial_hand.send(1.);closed=True;events.append(dict(time=self.now(),grasp=1.))
    if elapsed>report['times'][-1]+3:raise TimeoutError('descent controller completion unavailable')
    await pause(self,.02)
   response=done.result()
   if response.status!=4 or response.result.error_code:raise RuntimeError('descent arm action failed')
   if not closed:raise ValueError('closure interval missed')
   error=self.gate(goal)-np.array(report['path'][-1])[self.idx]
   if max(abs(error))>self.config['final_tolerance']:raise ValueError('descent final joint error')
   report.update(hand_commands=events,final_error_deg=np.degrees(error).tolist());completed=True
   return MoveToReady.Result(success=True,reason='Descent and right closure command completed; physical grasp success not measured')
  except Exception as e:
   report['reason']=f'{type(e).__name__}: {e}'
   return MoveToReady.Result(success=False,reason=report['reason'])
  finally:
   if not completed:
    if pending is not None:
     def cancel_late(f):
      try:
       if f.result().accepted:f.result().cancel_goal_async()
      except Exception:pass
     pending.add_done_callback(cancel_late)
    await cancel_children(self,[handle])
   report.update(success=completed,right_close_sent=closed,automatic_return=False)
   (OUT/'descent-result.json').write_text(json.dumps(report,indent=2))
   self.state='GraspTrialFinished' if completed else 'Stopped';self.reason='하강·닫힘 명령 완료; 파지 성공은 별도 확인 필요' if completed else report.get('reason','stopped')
 async def execute_one(self,goal):
  self.passive_left=None
  actual=self.gate(goal);self.passive_left=actual[:7].copy()
  self.descent_admitted=False;self.progressive_closed=False;self.close_started=None;self.close_commands=[];self.progressive_events=[]
  self.pregrasp_started=None
  self.config['_stationary_left_arm']=self.passive_left.tolist()
  self.config['progressive_descent']=True
  self.config['tracking_lookahead_s']=1. if self.config['tracking_grasp_policy'].get('preplan_entry') else 0.
  self.config['tracking_rotation_weight']=.05
  self.config['intercept_window']=self.intercept_zone
  self.config['intercept_approach_first']=True
  self.intercept_plan=None
  from deux_task_core.planning import scene_version
  self.config_version=scene_version(self.config)
  self.robot=planning_robot(self.config)
  if self.config['tracking_grasp_policy'].get('reusable_intercept_path'):
   self.seed_warmup={'reused_path':True,'warmup_s':0.}
  else:
   from initial_seed_trial import warm
   self.state='WarmingTrackingIK';self.reason='첫 접근 초기값 비교용 IK 준비 중; 명령 없음'
   warm_job=self.pool.submit(warm,self.config)
   while not warm_job.done():self.gate(goal);await pause(self,.03)
   self.seed_warmup=warm_job.result()
  self.repeat_deadline=time.monotonic()+300;self.used=False
  ready=await self.recover_ready(goal)
  if not ready.success:return ready
  self.used=False
  if self.config['tracking_grasp_policy'].get('reusable_intercept_path'):
   from reusable_intercept import prepare
   actual=await self.fresh_gate(goal)
   q=np.array(self.config['home_joints']);q[self.idx]=actual;q[:7]=self.passive_left
   self.state='PreparingFixedPath';self.reason='봉투 대기 전: 실제 준비자세에 저장 경로 1회 보정'
   job=self.pool.submit(prepare,self.config,q)
   while not job.done():self.gate(goal);await pause(self,.02)
   self.reusable_preparation=job.result()
  upper=await self.execute_once(goal)
  if not upper.success:
   (OUT/'descent-result.json').write_text(json.dumps(dict(success=False,reason=upper.reason,right_close_sent=bool(self.close_commands),closure_completed=self.progressive_closed,events=self.progressive_events,hand_commands=self.close_commands),indent=2))
   return upper
  # Descent is part of asynchronous tracking, with no stop-and-replan transition.
  result=MoveToReady.Result(success=self.progressive_closed,
    reason='Tracking descent and position-qualified right closure completed; grasp success not measured' if self.progressive_closed else 'Tracking ended before position-qualified closure; hand was not closed')
  self.state='GraspTrialFinished' if result.success else 'Stopped';self.reason=result.reason
  (OUT/'descent-result.json').write_text(json.dumps(dict(success=result.success,reason=result.reason,right_close_sent=bool(self.close_commands),closure_completed=self.progressive_closed,events=self.progressive_events,hand_commands=self.close_commands),indent=2))
  return result

 async def execute(self,goal):
  count=int(os.environ.get('DEUX_GRASP_REPEAT','1'))
  if not 1<=count<=5:raise ValueError('repeat count must be 1..5')
  history=[];run_dir=OUT/f'progressive-repeat-{time.time_ns()}';run_dir.mkdir()
  (OUT/'latest-repeat-dir.txt').write_text(str(run_dir))
  for attempt in range(count):
   self.used=False
   result=await self.execute_one(goal)
   summary=dict(attempt=attempt+1,success=result.success,reason=result.reason,
                target_id=self.target_id,right_close_sent=bool(self.close_commands),
                closure_completed=self.progressive_closed,time=self.now())
   history.append(summary)
   for name in ('result.json','descent-result.json','ready-right-hand.json'):
    source=OUT/name
    if source.exists():(run_dir/f'{attempt+1}-{name}').write_text(source.read_text())
   (run_dir/'summary.json').write_text(json.dumps(history,indent=2))
   # Any attempted closure may hold an object. Never open/return automatically.
   if self.close_commands or result.success:return result
   if result.reason!='Tracking ended before position-qualified closure; hand was not closed':
    return result
   if not self.last_report.get('controller_terminal_confirmed',False):
    return MoveToReady.Result(success=False,reason='repeat stopped: controller termination unconfirmed')
   self.seen_targets.add(self.target_id)
   if attempt+1==count:break
   self.state='Settling';self.reason=f'반복 시험 {attempt+1}/{count} 종료 · 정지 확인 후 준비자세 복귀'
   quiet=None;deadline=time.monotonic()+5.
   while True:
    self.gate(goal)
    values=dict(zip(self.arm.name,self.arm.velocity))
    if all(name in values for name in ARM_NAMES[7:]) and max(abs(values[name]) for name in ARM_NAMES[7:])<.05:
     quiet=quiet or time.monotonic()
     if time.monotonic()-quiet>=.5:break
    else:quiet=None
    if time.monotonic()>deadline:
     return MoveToReady.Result(success=False,reason='repeat stopped: right arm not settled')
    await pause(self,.02)
  self.state='Stopped';self.reason=f'반복 시험 {count}회 완료 · 닫힘 조건 미충족 · 오른손 개방 유지'
  return MoveToReady.Result(success=False,reason=self.reason)

if __name__=='__main__':
 rclpy.init();node=GraspTrial()
 try:rclpy.spin(node)
 finally:node.pool.shutdown(wait=False,cancel_futures=True);node.destroy_node();rclpy.try_shutdown()
