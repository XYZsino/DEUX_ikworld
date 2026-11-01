"""GUI-started upper-only waiting/rendezvous trial. No finger or descent commands."""
import json,time,hashlib
import numpy as np
import rclpy
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.action import MoveToReady
from deux_task_core.model import ARM_NAMES
from deux_task_core.planning import scene_version
from deux_task_runtime.common import pause,seconds,ros_time,cancel_children
from moving_upper_trial import MovingTrial,ROOT
from right_only_trial import RightOnlyClient,planning_robot
from async_upper import spline
from upper_rendezvous import (DEFAULTS,prepare_wait,following_path,fixed_observation,
                              arrival,dispatch_checks,Qualification,validate_settings,Settling)
from upper_x_alignment import CrossingWindow,crossing,select_point,build_alignment,admission as alignment_admission,AlignmentTooLate
OUT=ROOT/'.setup/upper-rendezvous'


class UpperWaitTrial(MovingTrial):
    def __init__(self):
        super().__init__()
        self.client=RightOnlyClient(self.client);self.passive_left=None
        self.policy=validate_settings(dict(DEFAULTS,**json.loads((ROOT/'calibration/upper_rendezvous.json').read_text())))
        self.template_path=ROOT/'calibration/reusable_intercept.json'
        self.template_bytes=self.template_path.read_bytes()
        record=json.loads(self.template_bytes);self.template=record['plan']
        self.fixed=np.asarray(self.template['fixed_interception']['pose']).copy()
        self.nominal_velocity=np.asarray(record['nominal_velocity'])
        direction=self.nominal_velocity[:2]/np.linalg.norm(self.nominal_velocity[:2])
        self.fixed[:2,3]-=direction*self.policy['upstream_offset_m']
        self.base_fixed=self.fixed.copy()
        self.grasp=np.asarray(self.template['grasp'])
        self.config['approach_height']=float(self.policy['upper_height_m'])
        self.config['tracking_lookahead_s']=0.
        self.config['tracking_grasp_policy']=json.loads((ROOT/'calibration/upper_wait_tracking_policy.json').read_text())
        self.config['intercept_window']=json.loads((ROOT/'calibration/intercept_window.json').read_text())
        self.config_version=scene_version(self.config)
        self.samples=[];self.events=[];self.intercept_pub=self.create_publisher(String,'/task/intercept_window',10)
        self.reason='상부 전용: 고정 높이 · 선행 대기 · 속도 합류 · 손/하강 명령 없음'
        OUT.mkdir(exist_ok=True)

    def gate(self,goal=None):
        if goal is not None and goal.is_cancel_requested:raise InterruptedError('GUI canceled')
        if self.estop:raise RuntimeError('e-stop active')
        self.hardware_gate();self.check_model_version()
        if time.monotonic()-self.external<.5:raise RuntimeError('competing arm command')
        actual=self.measured();limits=self.robot.limits[14:21]
        if np.any(actual[7:]<limits[:,0]-self.robot.joint_limit_tolerance) or np.any(actual[7:]>limits[:,1]+self.robot.joint_limit_tolerance):raise RuntimeError('right measured joint limit')
        if self.passive_left is not None and np.max(abs(actual[:7]-self.passive_left))>np.deg2rad(1):raise RuntimeError('passive left geometry changed')
        return actual

    def full(self,actual):
        q=np.asarray(self.config['home_joints']).copy();q[self.idx]=actual
        if self.passive_left is not None:q[:7]=self.passive_left
        return q

    async def worker(self,goal,fn,*args):
        job=self.pool.submit(fn,*args)
        while not job.done():self.gate(goal);await pause(self,.02)
        return job.result()

    def geometry(self):
        if self.camera is None:raise ValueError('camera information unavailable')
        return dict(camera=json.loads((ROOT/'calibration/zed2i_extrinsics.thor.json').read_text())['T_base_camera_optical'],projection=list(self.camera.p),width=self.camera.width,height=self.camera.height,roi=json.loads((ROOT/'.setup/single-track-roi.json').read_text()))

    def evidence(self):
        tid,obj,v,stamp=self.tracked_object(allow_hold=False)
        obj,v,raw_z=fixed_observation(obj,v,self.grasp,self.fixed,None)
        if np.dot(v[:2],self.nominal_velocity[:2])<=0:raise ValueError('belt direction reversed')
        return tid,obj,v,stamp,raw_z

    async def settled(self,goal,point):
        until=time.monotonic()+5.;detector=Settling(point,self.policy['settle_s'])
        while time.monotonic()<until:
            actual=self.gate(goal);ts=seconds(self.arm.header.stamp);q=self.full(actual)
            done,event=detector.update(ts,actual[7:],self.robot.ee_fk('right',q)[:3,3])
            if event:
                event.update(stage='settling',time=self.now(),raw_joint_velocity=list(self.arm.velocity))
                self.events.append(event)
            if done:return
            await pause(self,.025)
        raise ValueError('upper waiting pose not settled')

    async def dispatch(self,goal,plan,start_at,monitor=None):
        pending=None;handle=None;completed=False
        logged_stamp=None
        q0=np.asarray(plan['path'][0])[self.idx]
        if np.max(abs(self.gate(goal)-q0))>.015:raise ValueError('start changed after validation')
        curve=spline(plan);end=float(plan['times'][-1])
        tr=JointTrajectory(joint_names=list(ARM_NAMES),points=[JointTrajectoryPoint(
            positions=np.array(q)[self.idx].tolist(),velocities=np.array(v)[self.idx].tolist(),
            time_from_start=ros_time(float(t),Duration)) for t,q,v in zip(plan['times'],plan['path'],plan['velocities'])])
        tr.header.stamp=ros_time(start_at)
        if self.now()>start_at-.2:raise ValueError('late dispatch')
        try:
            pending=self.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=tr))
            handle=await self.future(pending,goal,seconds=.4)
            if not handle.accepted:raise RuntimeError('controller rejected')
            if self.now()>=start_at:raise RuntimeError('controller accepted too late')
            done=handle.get_result_async();deadline=time.monotonic()+max(0,start_at-self.now())+end+3.
            while not done.done():
                actual=self.gate(goal);ts=seconds(self.arm.header.stamp)
                if ts>self.now():raise RuntimeError('future feedback timestamp')
                elapsed=ts-start_at
                if np.max(abs(actual-curve(np.clip(elapsed,0,end))[self.idx]))>self.config['tracking_tolerance']:raise RuntimeError('joint tracking error')
                stages=plan.get('right_approach',{}).get('stages',[])
                if stages:
                    stage=next((s for s in stages if elapsed<=s['end_s']),stages[-1])
                    self.state=stage['name']
                    self.reason={'RightEscape':'몸통 기준 오른쪽으로 먼저 이동 중','RaiseOutsideConveyor':'컨베이어 바깥에서 상승 중','AdvanceOnRight':'오른쪽 통로로 전진 중','EnterUpperWait':f'봉투 위 {self.config["approach_height"]*100:.0f}cm 대기점으로 진입 중'}[stage['name']]
                if monitor is not None:monitor(ts,self.full(actual),elapsed)
                elif ts!=logged_stamp:
                    q=self.full(actual);self.samples.append(dict(stage='preposition',feedback_stamp=ts,time=self.now(),elapsed_s=elapsed,q=q.tolist(),hand=self.robot.ee_fk('right',q)[:3,3].tolist()))
                    logged_stamp=ts
                if time.monotonic()>deadline:raise TimeoutError('controller completion timeout')
                await pause(self,.02)
            response=done.result()
            if response.status!=4 or response.result.error_code:raise RuntimeError('controller trajectory failed')
            if np.max(abs(self.gate(goal)-np.asarray(plan['path'][-1])[self.idx]))>self.config['final_tolerance']:raise RuntimeError('final joint error')
            completed=True
        finally:
            if not completed:
                if pending is not None:
                    def late(f):
                        try:
                            h=f.result()
                            if h.accepted:h.cancel_goal_async()
                        except Exception:pass
                    pending.add_done_callback(late)
                await cancel_children(self,[handle])
                self.used=True  # No recovery or new task after an unconfirmed cancellation.

    async def align_x(self,goal,tid,candidate,plan,geometry):
        self.state='AlignCrossingX';self.reason=f'예상 통과 X={candidate[0,3]*100:.1f}cm로 상부 정렬 중'
        lost=[];last=None
        def monitor(ts,q,elapsed):
            nonlocal last
            if ts==last:return
            last=ts;hand=self.robot.ee_fk('right',q)[:3,3]
            self.samples.append(dict(stage='x_alignment',time=self.now(),feedback_stamp=ts,elapsed_s=elapsed,q=q.tolist(),hand=hand.tolist(),target_x_m=float(candidate[0,3])))
            try:
                current,obj,v,stamp,raw_z=self.evidence()
                if current!=tid:raise ValueError('X 정렬 중 대상 ID 변경')
                x,at=crossing(candidate,obj,v,self.grasp,stamp,self.now())
                if abs(x-candidate[0,3])>.01:raise ValueError('X 정렬 중 예상 통과 X 편차 1cm 초과')
                if self.geometry()!=geometry:raise ValueError('X 정렬 중 ROI/카메라 변경')
            except ValueError as e:
                if elapsed<0:raise
                if not lost:lost.append(str(e))
        try:
            await self.dispatch(goal,plan,self.now()+.4,monitor)
            self.fixed=candidate.copy()
            upper=self.fixed[:3,3].copy();upper[2]+=self.config['approach_height']
            await self.settled(goal,upper)
        except Exception as e:
            # An execution/cancellation failure must never return to the target-selection loop.
            raise RuntimeError('X 정렬 실행 중단: '+str(e)) from e
        return lost

    async def execute(self,goal):
        self.used=True;self.samples=[];self.events=[];self.target_id='';report={};result=MoveToReady.Result(success=False)
        try:
            if self.template_path.read_bytes()!=self.template_bytes:raise ValueError('fixed path changed since startup')
            actual=self.gate(goal);self.passive_left=actual[:7].copy()
            self.config['_stationary_left_arm']=self.passive_left.tolist();self.config_version=scene_version(self.config)
            self.robot=planning_robot(self.config)
            q=self.full(actual);endpoint=q.copy();endpoint[14:21]=np.asarray(self.template['path'][-1])[14:21]
            upper=self.fixed[:3,3].copy();upper[2]+=self.config['approach_height']
            self.state='PlanningUpperWait';self.reason='오른쪽 직선 이탈 → 바깥 상승 → 우측 전진 → 상부 진입 경로 검증 중'
            plan=await self.worker(goal,prepare_wait,self.config,q,endpoint,self.fixed)
            report['preposition']=plan;self.state='MovingUpperWait';self.reason=f'고정 파지점 위 {self.config["approach_height"]*100:.0f}cm로 이동 중'
            await self.dispatch(goal,plan,self.now()+.75)
            await self.settled(goal,upper)
            deadline=time.monotonic()+self.policy['wait_timeout_s'];last_attempt=None
            x_window=CrossingWindow();aligned_target=None;skipped=set();wait_reason='ROI 진입 후 예상 통과 X 안정화 대기'
            while True:
                self.gate(goal)
                if time.monotonic()>deadline:raise TimeoutError('no feasible target before wait timeout')
                self.state='WaitingFixedPoint' if aligned_target else 'WaitingCrossingX';self.reason=wait_reason
                try:
                    tid,obj,v,stamp,raw_z=self.evidence();geometry=self.geometry()
                    if (tid,stamp)==last_attempt:await pause(self,.03);continue
                    last_attempt=(tid,stamp)
                    if tid in skipped:
                        await pause(self,.03);continue
                    x,arrival_at=crossing(self.base_fixed,obj,v,self.grasp,stamp,self.now())
                    if aligned_target!=tid:
                        stable=x_window.update(tid,stamp,x)
                        if stable is None:
                            wait_reason=f'예상 통과 X 안정화 대기 · X={x*100:.1f}cm';await pause(self,.03);continue
                        candidate=select_point(self.base_fixed,stable,self.config)
                        self.target_id=tid
                        q=self.full(self.gate(goal));hand=self.robot.ee_fk('right',q)[:3,3]
                        if abs(hand[0]-stable)>.004:
                            self.state='PlanningXAlignment';self.reason=f'X {hand[0]*100:.1f}→{stable*100:.1f}cm 정렬 및 합류·감속 사전 검증'
                            alignment=await self.worker(goal,build_alignment,self.config,q,candidate,v,self.policy)
                            current,obj,v,stamp,raw_z=self.evidence()
                            if current!=tid:skipped.add(tid);raise ValueError('X 계획 중 대상 변경')
                            at,needed=alignment_admission(alignment,candidate,self.grasp,obj,v,stamp,self.now(),geometry,self.policy)
                            self.events.append(dict(stage='x_alignment_planned',time=self.now(),target_id=tid,source_stamp=stamp,old_x_m=float(self.fixed[0,3]),target_x_m=stable,arrival_at=at,required_time_s=needed,plan=alignment))
                            lost=await self.align_x(goal,tid,candidate,alignment,geometry)
                            if lost:skipped.add(tid);raise ValueError(lost[0])
                            self.events.append(dict(stage='x_alignment_settled',time=self.now(),target_id=tid,target_x_m=stable))
                        else:
                            self.fixed=candidate.copy()
                            self.events.append(dict(stage='x_alignment_within_deadband',time=self.now(),target_id=tid,target_x_m=stable,measured_x_m=float(hand[0])))
                        aligned_target=tid;wait_reason='X 정렬 완료 · Y 방향 합류 시각 대기'
                        # Always reacquire evidence after the physical alignment/settling interval.
                        await pause(self,.02);continue
                    at,lateral=arrival(self.fixed,obj,v,self.grasp,stamp,self.now())
                    eta=at-self.now()
                    self.events.append(dict(stage='arrival_prediction',time=self.now(),target_id=tid,source_stamp=stamp,arrival_at=at,lateral_m=lateral,raw_grasp_z_m=raw_z,fixed_grasp_z_m=float(self.fixed[2,3]),target_x_m=float(self.fixed[0,3]),predicted_x_m=x))
                    if lateral>self.policy['xy_m']/2:
                        skipped.add(tid);wait_reason=f'정렬 후 이동선 편차 {lateral*100:.1f}cm > 1cm · 이번 봉투 건너뜀';self.events.append(dict(stage='admission_rejected',time=self.now(),reason=wait_reason));continue
                    if eta<1.3:
                        skipped.add(tid);wait_reason=f'합류 시간 부족: 도착 {eta:.2f}초 · 이번 봉투 건너뜀';self.events.append(dict(stage='admission_rejected',time=self.now(),reason=wait_reason));continue
                    if eta>3.:
                        wait_reason=f'X 정렬 완료 · 도착 {eta:.2f}초 · 이동선 오차 {lateral*100:.1f}cm';await pause(self,.03);continue
                    q=self.full(self.gate(goal));self.state='PlanningVelocityMerge';self.reason='고정 높이 합류·추종·감속 경로 검증 중'
                    plan=await self.worker(goal,following_path,self.config,q,self.fixed,v,self.policy)
                    live_tid,obj,v,stamp,raw_z=self.evidence()
                    if live_tid!=tid:raise ValueError('target changed during planning')
                    start,at,lateral=dispatch_checks(plan,self.fixed,self.grasp,obj,v,stamp,self.now(),geometry,self.policy)
                    break
                except ValueError as e:
                    if isinstance(e,AlignmentTooLate):skipped.add(tid)
                    wait_reason=str(e);self.reason=wait_reason;self.events.append(dict(time=self.now(),stage='admission_rejected',reason=str(e)))
                    await pause(self,.05)
            self.target_id=tid;qual=Qualification(self.policy)
            source=obj.copy();source_v=v.copy();source_stamp=stamp;last_feedback=None;last_now=self.now()
            report.update(target_id=tid,fixed_pose=self.fixed.tolist(),fixed_grasp_z_m=float(self.fixed[2,3]),upper_z_m=float(upper[2]),
                source='existing fixed grasp height/orientation; waiting point shifted upstream along recorded belt direction',
                upstream_offset_m=self.policy['upstream_offset_m'],
                start_at=start,arrival_at=at,merge_at=start+plan['merge_s'],brake_at=start+plan['brake_start_s'],
                plan=plan,policy=self.policy,grasp=self.grasp.tolist(),source_stamp=stamp,source_object=source.tolist())
            self.intercept_pub.publish(String(data=json.dumps(dict(frame='base_link',active=True,validated=True,target_id=tid,
                encounter_position=self.fixed[:3,3].tolist(),arrival_in_s=at-self.now(),tracking_lookahead_s=0.,reason='고정점 기준 도착 / 상부 합류'))))
            def monitor(ts,q,elapsed):
                nonlocal last_feedback,last_now
                now=self.now()
                if now<last_now:raise RuntimeError('clock moved backwards')
                last_now=now
                if ts==last_feedback:return
                if last_feedback is not None and ts<last_feedback:raise RuntimeError('feedback clock moved backwards')
                last_feedback=ts;hand=self.robot.ee_fk('right',q)[:3,3]
                try:
                    current,obj,v,stamp,raw_z=self.evidence()
                    if current!=tid:raise ValueError('target identity lost')
                    if not 0<=ts-stamp<=self.policy['max_observation_age_s']:raise ValueError('timestamp mismatch')
                    live=(obj@self.grasp)[:3,3]+v[:3]*(ts-stamp)
                    expected=(source@self.grasp)[:3,3]+source_v[:3]*(ts-source_stamp)
                    if np.linalg.norm(live[:2]-expected[:2])>.01:raise ValueError('prediction drift >1cm')
                    if np.linalg.norm(v[:2]-source_v[:2])>.005:raise ValueError('velocity drift >0.5cm/s')
                    if self.geometry()!=geometry:raise ValueError('ROI/camera geometry changed')
                    from deux_task_core.live_continuous_preview import roi_contains
                    if not roi_contains(geometry,[live]):raise ValueError('target left ROI')
                    upper_bag=live.copy();upper_bag[2]=upper[2]
                    e=qual.update(ts,hand,upper_bag,v,eligible=plan['merge_s']<=elapsed<=plan['brake_start_s'])
                    if e:
                        e.update(time=now,elapsed_s=elapsed,source_stamp=stamp,raw_grasp_z_m=raw_z,fixed_grasp_z_m=float(self.fixed[2,3]),
                                 arrival_at=arrival(self.fixed,obj,v,self.grasp,stamp,now)[0],merge_at=start+plan['merge_s'],q=q.tolist())
                        self.samples.append(e)
                except ValueError as error:
                    if elapsed<0:raise ValueError('evidence lost before scheduled merge: '+str(error))
                    qual.invalid=True;qual.since=None
                    self.events.append(dict(time=now,stage='evidence_lost',reason=str(error)))
                    self.reason='관측 조건 소실 · 성공 판정 중지 · 검증된 상부 경로와 감속 종료'
                if not qual.invalid:self.reason='상부 속도 합류·추종 오차 측정 중' if elapsed<plan['brake_start_s'] else '검증된 감속 경로로 정지 중'
            self.state='UpperVelocityMerge'
            await self.dispatch(goal,plan,start,monitor)
            await self.settled(goal,self.robot.ee_fk('right',np.asarray(plan['path'][-1]))[:3,3])
            report['measured_stop_confirmed']=True
            # Qualification is distinct from successful controller completion.
            result.success=bool(qual.passed and not qual.invalid)
            result.reason=('Upper XY/Z/relative-speed criteria held for 0.5s; braking completed; no descent or hand command' if result.success else 'Upper motion/braking completed, but continuous relative-motion qualification failed')
        except Exception as e:result.reason=f'{type(e).__name__}: {e}'
        finally:
            self.state='Finished' if result.success else 'Stopped';self.reason=result.reason
            self.intercept_pub.publish(String(data=json.dumps(dict(frame='base_link',active=False,validated=False,reason=self.reason))))
            report.update(success=result.success,reason=result.reason,events=self.events,samples=self.samples,
                          finger_commands=False,descent_commands=False,template_sha256=hashlib.sha256(self.template_bytes).hexdigest())
            path=OUT/f'{time.time_ns()}-result.json';path.write_text(json.dumps(report,indent=2));(OUT/'latest.json').write_text(json.dumps(report,indent=2))
        return result

if __name__=='__main__':
    rclpy.init();node=UpperWaitTrial()
    try:rclpy.spin(node)
    finally:node.pool.shutdown(wait=False,cancel_futures=True);node.destroy_node();rclpy.try_shutdown()
