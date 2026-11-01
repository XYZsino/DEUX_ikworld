"""Dedicated GUI-cancelable right-arm ready trial. Never commands fingers or tasks."""
import json,time,uuid,os
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.action import MoveToReady
from deux_task_msgs.msg import TaskStatus,MotionPlan
from deux_task_core.model import RobotModel,JOINT_NAMES,ARM_NAMES
from deux_task_runtime.common import RuntimeNode,ExclusiveAction,LATCHED,current,joint_array,ros_time,pause,cancel_children
from deux_task_runtime.motion_review import MotionReview,Ticket,preview
from deux_task_runtime.trajectory import validate_limits,sample
ROOT=Path('/home/xyz_thor/DEUX_ikworld')
OUT=ROOT/'.setup/ready-arm-trial'
class Trial(RuntimeNode):
 def __init__(self):
  super().__init__('ready_arm_trial',observe_status=True,parameter_overrides=[Parameter('config_file',value=str(ROOT/'src/deux_task_bringup/config/installation.yaml'))])
  self.robot=RobotModel(self.config['robot']);self.idx=[JOINT_NAMES.index(n) for n in ARM_NAMES]
  self.target=np.array(json.loads((ROOT/'.setup/ready-minus30/candidate.json').read_text())['chosen']['new_home'])[14:21]
  self.arm=None;self.arm_receive=0.;self.estop=False;self.external=0.;self.state='Idle';self.reason='Right-arm ready trial only; GUI approval required';self.used=False
  self.create_subscription(JointState,'/arm/joint_states',self.on_arm,qos_profile_sensor_data)
  self.create_subscription(Bool,'/deux/estop',lambda m:setattr(self,'estop',m.data),10)
  self.create_subscription(JointTrajectory,'/arm/deux_arm_controller/joint_trajectory',lambda m:setattr(self,'external',time.monotonic()),10)
  self.client=ActionClient(self,FollowJointTrajectory,'/arm/deux_arm_controller/follow_joint_trajectory',callback_group=self.group)
  self.review=MotionReview(self);self.pub=self.create_publisher(TaskStatus,'/task/status',LATCHED)
  self.action=ExclusiveAction(self,MoveToReady,'/task/move_to_ready',self.execute,self.admit)
  self.create_timer(.1,self.status,callback_group=self.group)
  OUT.mkdir(exist_ok=True)
 def on_arm(self,m):self.arm=m;self.arm_receive=time.monotonic()
 def status(self):
  m=TaskStatus(state=self.state,reason=self.reason,active=self.action.busy,workflow='ReadyArmTrial');m.header.stamp=ros_time(self.now());self.pub.publish(m)
 def measured(self):
  if self.arm is None or time.monotonic()-self.arm_receive>self.config['max_joint_age'] or not current(self.arm.header.stamp,self.now(),self.config['max_joint_age']):raise RuntimeError('arm feedback stale')
  return joint_array(self.arm,ARM_NAMES)
 def gate(self,goal=None):
  if goal is not None and goal.is_cancel_requested:raise InterruptedError('GUI canceled')
  if self.estop:raise RuntimeError('e-stop active')
  self.hardware_gate();self.check_model_version()
  if time.monotonic()-self.external<.5:raise RuntimeError('competing arm trajectory command')
  q=self.measured();bounds=self.robot.limits[self.idx]
  if np.any(q<bounds[:,0]-self.robot.joint_limit_tolerance) or np.any(q>bounds[:,1]+self.robot.joint_limit_tolerance):raise RuntimeError('measured joint limit')
  return q
 def admit(self,request):
  if self.used:raise RuntimeError('one-shot trial already executed; restart to replan')
  self.gate()
  if not self.client.server_is_ready():raise RuntimeError('arm controller action unavailable')
  if self.count_publishers('/task/status')!=1:raise RuntimeError('another task runtime is active')
 async def future(self,f,goal,seconds=3.):
  deadline=time.monotonic()+seconds
  while not f.done():
   self.gate(goal)
   if time.monotonic()>deadline:raise TimeoutError('controller response timeout')
   await pause(self)
  return f.result()
 async def execute(self,goal):
  handle=None;pending=None;report={};result=MoveToReady.Result()
  try:
   start=self.gate(goal);target=start.copy();target[7:14]=self.target
   from ready_conveyor_geometry import clearance
   path=json.loads(getattr(self,'path_file',ROOT/'.setup/ready-minus30-path/path.json').read_text())
   waypoints=np.array(path['right_waypoints'])
   if max(abs(start[7:]-waypoints[0]))>.03:raise RuntimeError('measured start differs from conveyor plan; replan required')
   knots=[start.copy()]
   for right in waypoints[1:]:
    row=start.copy();row[7:]=right;knots.append(row)
   times=[];rows=[];offset=0.
   for a0,b0 in zip(knots[:-1],knots[1:]):
    delta=abs(b0-a0);v=np.minimum(self.robot.velocity[self.idx],.25);a=np.minimum(self.robot.acceleration[self.idx],.3)
    segment=max(1.,float(np.max(1.5*delta/v)),float(np.sqrt(np.max(6*delta/a))))+.2
    tt=np.linspace(0,segment,int(np.ceil(segment/.02))+1);u=tt/segment;qq=a0+(3*u*u-2*u*u*u)[:,None]*(b0-a0)
    begin=0 if not times else 1;times.extend((tt[begin:]+offset).tolist());rows.extend(qq[begin:]);offset+=segment
   t=np.array(times);q=np.array(rows);duration=float(t[-1])
   tr=JointTrajectory(joint_names=list(ARM_NAMES),points=[JointTrajectoryPoint(positions=row.tolist(),time_from_start=ros_time(float(tm),Duration)) for tm,row in zip(t,q)])
   validate_limits(tr,ARM_NAMES,self.robot)
   q28=np.tile(np.array(self.config['home_joints']),(len(q),1));q28[:,self.idx]=q
   # Grasp-frame envelopes are independent of fingers; no hand command is sent.
   min_clearance=10.
   for row in q28:
    self.robot.clearance(row,self.config.get('obstacles',[]),self.config['collision_margin'])
    distance=clearance(self.robot,row);min_clearance=min(min_clearance,distance)
    if distance<0:raise RuntimeError('conveyor screening collision')
   plan=MotionPlan(plan_id=uuid.uuid4().hex,stage='ReadyMinus30Trial',scene_version=self.config_version,trajectory=tr)
   self.review.data=preview(self.robot,plan,q28[0],t,q28);self.review.ticket=Ticket(plan.plan_id,self.review.data['digest'],900.)
   self.review.data.update(request_id=self.review.ticket.request_id,warning='Measured finite conveyor surfaces: 6cm arm / 5cm wrist-hand capsules + 1.5cm margin + 1.75cm surface uncertainty. Right arm only; no hand commands. Unobserved geometry is not verified.')
   report=dict(start_deg=np.rad2deg(start).tolist(),target_deg=np.rad2deg(target).tolist(),duration_s=duration,plan_id=plan.plan_id,min_conveyor_clearance_m=min_clearance,screening=path['screening'])
   (OUT/'plan.json').write_text(json.dumps(report,indent=2));self.state='AwaitingApproval';self.reason='준비자세 · 컨베이어 회피 · GUI 승인 대기'
   # FK preview construction is CPU bound; drain fresh feedback before admission.
   await pause(self,.05)
   automatic=os.environ.get('DEUX_READY_AUTO')=='1'
   while not automatic and self.review.ticket.decision is None:
    if np.max(abs(self.gate(goal)-start))>.05:raise RuntimeError('start changed; new preview required')
    if time.monotonic()>self.review.ticket.deadline:raise TimeoutError('review expired; replan required')
    self.review.publish('pending',self.reason);await pause(self,.1)
   if not automatic and self.review.ticket.decision!='approve':raise InterruptedError('operator rejected')
   if np.max(abs(self.gate(goal)-start))>.05:raise RuntimeError('start changed after approval')
   self.used=True;self.review.publish('approved');self.state='Executing';self.reason='승인된 준비자세 경로 실행 중'
   start_at=self.now()+.75;tr.header.stamp=ros_time(start_at)
   pending=self.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=tr));handle=await self.future(pending,goal)
   if not handle.accepted:raise RuntimeError('arm controller rejected')
   if self.now()>=start_at:raise RuntimeError('late acceptance')
   done=handle.get_result_async();deadline=time.monotonic()+duration+10;trace=[]
   while not done.done():
    actual=self.gate(goal);elapsed=self.now()-start_at;err=abs(actual-sample(t,q,max(0.,elapsed)));trace.append([elapsed,*actual.tolist()])
    if np.max(err)>self.config['tracking_tolerance']:raise RuntimeError('ready trial tracking error')
    if time.monotonic()>deadline:raise TimeoutError('controller completion timeout')
    await pause(self,.02)
   response=done.result()
   if response.status!=4 or response.result.error_code:raise RuntimeError('controller failed: '+response.result.error_string)
   final=self.gate(goal);report['final_deg']=np.rad2deg(final).tolist();report['error_deg']=np.rad2deg(final-target).tolist();report['trace']=trace
   if max(abs(final-target))>self.config['final_tolerance']:raise RuntimeError('final ready error')
   handle=None;pending=None;result.success=True;result.reason='Right-arm ready motion completed; no task or hand command started';self.state='Ready'
  except Exception as e:
   result.success=False;result.reason=f'{type(e).__name__}: {e}';self.state='Stopped'
  finally:
   if pending is not None and handle is None:
    def late(f):
     try:
      h=f.result()
      if h.accepted:h.cancel_goal_async()
     except Exception:pass
    pending.add_done_callback(late)
   await cancel_children(self,[handle])
   self.review.publish('closed',result.reason);self.review.ticket=None;self.review.data=None;self.reason=result.reason
   report.update(success=result.success,reason=result.reason);(OUT/'result.json').write_text(json.dumps(report,indent=2))
  return result
if __name__=='__main__':
 rclpy.init();node=Trial()
 try:rclpy.spin(node)
 finally:node.destroy_node();rclpy.shutdown()
