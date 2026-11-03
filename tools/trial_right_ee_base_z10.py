"""User-requested arm-only ready-pose comparison. Each move requires GUI approval."""
import json,time,uuid,sys,socket,struct
from pathlib import Path
import numpy as np
import rclpy
from rclpy.parameter import Parameter,parameter_value_to_python
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters,SetParametersAtomically
from std_msgs.msg import Bool,Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.msg import TaskStatus,MotionPlan
from deux_task_core.model import RobotModel,JOINT_NAMES,ARM_NAMES,HAND_NAMES
from deux_task_runtime.common import RuntimeNode,LATCHED,current,joint_array,ros_time
from deux_task_runtime.motion_review import MotionReview,Ticket,preview
from deux_task_runtime.trajectory import validate_limits,sample

root=Path('/home/xyz_thor/DEUX_ikworld');out=root/'.setup/ee-base-z10-trial';out.mkdir(exist_ok=True)
rclpy.init();node=RuntimeNode('ee_base_z10_trial',observe_state=True,observe_status=True,parameter_overrides=[Parameter('config_file',value=str(root/'src/deux_task_bringup/config/installation.yaml'))])
robot=RobotModel(node.config['robot']);idx=[JOINT_NAMES.index(x) for x in ARM_NAMES];state={};samples=[];efforts=[];external=[];estop=[False]
def arm(m):state['arm']=m;samples.append((time.monotonic(),joint_array(m,ARM_NAMES).tolist()))
def effort(m):efforts.append((time.monotonic(),list(m.data)))
subs=[node.create_subscription(JointState,'/arm/joint_states',arm,qos_profile_sensor_data),node.create_subscription(JointState,'/task/hand_state',lambda m:state.update(hand=m),qos_profile_sensor_data),node.create_subscription(TaskStatus,'/task/status',lambda m:state.update(task=m),LATCHED),node.create_subscription(Float64MultiArray,'/arm/deux_arm_effort_controller/commands',effort,10),node.create_subscription(JointTrajectory,'/arm/deux_arm_controller/joint_trajectory',lambda m:external.append(time.monotonic()),10),node.create_subscription(Bool,'/deux/estop',lambda m:estop.__setitem__(0,m.data),10)]
client=ActionClient(node,FollowJointTrajectory,'/arm/deux_arm_controller/follow_joint_trajectory');get=node.create_client(GetParameters,'/deux_gravity/get_parameters');setp=node.create_client(SetParametersAtomically,'/deux_gravity/set_parameters_atomically');review=MotionReview(node)
report={'runs':[],'commands':'right EE base Z +0.10m, preserve orientation; other arm joints held, no hand commands','comparison':'single joint trial; effort unchanged','error':None};saved=None;modified=False;handle=None;pending=None
keys=['scale_joint','left.bias','right.bias','left.friction_c','right.friction_c'];control=['mode','kp_scale','kd_scale','contact_enabled','bias_autozero','autozero_now']

def save():
 (out/'report.json').write_text(json.dumps(report,indent=2));(out/'trace.json').write_text(json.dumps({'arm':samples,'effort':efforts}))
can_socket=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
can_socket.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FD_FRAMES,1)
can_socket.bind(('can0',));can_socket.setblocking(False);can_seen={}
def poll_can():
 while True:
  try:data=can_socket.recv(72)
  except BlockingIOError:break
  cid=struct.unpack_from('=I',data)[0]
  if cid in range(0x11,0x18) and data[4]>=8:can_seen[cid]=time.monotonic()
def spin(sec=.05):
 end=time.monotonic()+sec
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.02);poll_can()
def call(c,req):
 if not c.wait_for_service(timeout_sec=2):raise RuntimeError('service missing '+c.srv_name)
 f=c.call_async(req);end=time.monotonic()+3
 while not f.done() and time.monotonic()<end:spin()
 if not f.done():raise RuntimeError('service response missing '+c.srv_name)
 return f.result()
def params(names):
 r=call(get,GetParameters.Request(names=names));return {k:parameter_value_to_python(v) for k,v in zip(names,r.values)}
def set_params(values):
 r=call(setp,SetParametersAtomically.Request(parameters=[Parameter(k,value=v).to_parameter_msg() for k,v in values.items()]))
 if not r.result.successful:raise RuntimeError(r.result.reason)
def measured():
 m=state.get('arm')
 if m is None or not current(m.header.stamp,node.now(),node.config['max_joint_age']):raise RuntimeError('arm feedback stale')
 return joint_array(m,ARM_NAMES)
def gate():
 poll_can()
 missing=[hex(i) for i in range(0x11,0x18) if time.monotonic()-can_seen.get(i,0)>.2]
 if missing:raise RuntimeError('right CAN feedback stale: '+str(missing))
 if estop[0]:raise RuntimeError('e-stop active')
 node.hardware_gate();node.robot_ready()
 m=state.get('task')
 if m is None or not current(m.header.stamp,node.now(),1.) or m.active:raise RuntimeError('BT must remain inactive')
 if external and time.monotonic()-external[-1]<.5:raise RuntimeError('another arm trajectory publisher sent a command')
 q=measured();limits=robot.limits[idx]
 if np.any(q<limits[:,0]-robot.joint_limit_tolerance) or np.any(q>limits[:,1]+robot.joint_limit_tolerance):raise RuntimeError('measured arm joint limit')
 return q

def run(label,off=False):
 global handle,pending,modified
 spin(.3);gate()
 from deux_task_core.ik_backends import attach_backend
 from scipy.spatial.transform import Rotation
 attach_backend(robot,node.config)
 spin(.4)
 # Compile using current state before freezing the reviewed start pose.
 full=np.array(node.config['home_joints']);full[idx]=measured()
 robot.ik('right',robot.ee_fk('right',full),full)
 spin(.3);start=gate();full[idx]=start
 if 'hand' not in state:raise RuntimeError('hand feedback unavailable')
 h=joint_array(state['hand'],HAND_NAMES)
 full[[JOINT_NAMES.index(n) for n in HAND_NAMES]]=h
 initial=robot.ee_fk('right',full);goal_pose=initial.copy();goal_pose[2,3]+=.10
 solved=robot.ik('right',goal_pose,full)
 actual_pose=robot.ee_fk('right',solved)
 position_error=float(np.linalg.norm(actual_pose[:3,3]-goal_pose[:3,3]))
 rotation_error=float(Rotation.from_matrix(goal_pose[:3,:3]@actual_pose[:3,:3].T).magnitude())
 if position_error>.005 or rotation_error>np.deg2rad(2):raise RuntimeError(f'EE trial IK residual: {position_error}m {rotation_error}rad')
 report['cartesian_goal']=dict(start=initial.tolist(),target=goal_pose.tolist(),predicted=actual_pose.tolist(),position_error_m=position_error,rotation_error_rad=rotation_error)
 target=solved[idx]
 print('IK_GOAL '+json.dumps(report['cartesian_goal']),flush=True)
 spin(.3)

 # Conservative position-only cubic trajectory; preserve original joint bounds.
 delta=abs(target-start);v=np.minimum(robot.velocity[idx],.25);a=np.minimum(robot.acceleration[idx],.3)
 duration=max(3.,float(np.max(1.5*delta/v)),float(np.sqrt(np.max(6*delta/a))))+.2
 t=np.linspace(0,duration,int(np.ceil(duration/.02))+1);u=t/duration;q=start+(3*u*u-2*u*u*u)[:,None]*(target-start)
 tr=JointTrajectory(joint_names=list(ARM_NAMES),points=[JointTrajectoryPoint(positions=row.tolist(),time_from_start=ros_time(float(tm),Duration)) for tm,row in zip(t,q)])
 validate_limits(tr,ARM_NAMES,robot)
 q28=np.tile(np.array(node.config['home_joints']), (len(q),1));q28[:,idx]=q
 if 'hand' in state:
  h=joint_array(state['hand'],HAND_NAMES);q28[:,7:14]=h[:7];q28[:,21:28]=h[7:]
 # Arm bounds already checked by validate_limits; finger positions are read-only.
 # Check configured geometry separately, without imposing finger command limits.
 if node.config['obstacles'] or node.config['attached_radius']:
  raise RuntimeError('this arm-only trial requires a full environment geometry checker')
 for row in q28:
  fk=robot.fk(row);spheres=[(item['link'],(fk[item['link']]@np.r_[item['center'],1.])[:3],item['radius']) for item in robot.spheres]
  for i,(name,center,radius) in enumerate(spheres):
   for other,point,other_radius in spheres[:i]:
    if name==other or frozenset((name,other)) in robot.ignore:continue
    if np.linalg.norm(center-point)-radius-other_radius<node.config['collision_margin']:
     raise RuntimeError('hand sphere collision clearance')
 plan=MotionPlan(plan_id=uuid.uuid4().hex,stage='EEBaseZ10',scene_version=node.config_version,trajectory=tr)
 review.data=preview(robot,plan,q28[0],t,q28);review.ticket=Ticket(plan.plan_id,review.data['digest'],300.)
 review.data.update(request_id=review.ticket.request_id,warning=f'{label}: Right EE base Z +10cm; current orientation preserved. Left arm and hands unchanged. '+('Feedforward will be set to zero after approval.' if off else 'Current gravity compensation.'))
 print('AWAITING_GUI_APPROVAL '+label,flush=True)
 while review.ticket.decision is None:
  gate();review.publish('pending',label);spin(.2)
  if time.monotonic()>review.ticket.deadline:raise RuntimeError('comparison preview expired')
 if review.ticket.decision!='approve':raise RuntimeError('operator rejected test')
 if np.max(abs(gate()-start))>.05:raise RuntimeError('arm moved during preview; replan required')
 if off:
  modified=True;set_params({k:[0.]*7 for k in keys});spin(.4)
  recent=[e for tm,e in efforts if time.monotonic()-tm<.25]
  if not recent or max(abs(x) for row in recent for x in row)>1e-6:raise RuntimeError('OFF effort is not zero')
  if np.max(abs(gate()-start))>.05:raise RuntimeError('arm drift after OFF; not dispatching stale path')
 else:
  if modified:set_params(saved);modified=False;spin(.4)
  if np.max(abs(gate()-start))>.05:raise RuntimeError('arm drift after restoring ON; replan required')
 review.publish('approved',label);review.ticket=None;review.data=None
 if not client.wait_for_server(timeout_sec=2):raise RuntimeError('arm action unavailable')
 start_at=node.now()+.75;tr.header.stamp=ros_time(start_at)
 pending=client.send_goal_async(FollowJointTrajectory.Goal(trajectory=tr))
 end=time.monotonic()+3
 while not pending.done() and time.monotonic()<end:spin()
 if not pending.done():raise RuntimeError('arm acceptance pending')
 handle=pending.result()
 if not handle.accepted:raise RuntimeError('arm goal rejected')
 result=handle.get_result_async()
 if node.now()>=start_at:raise RuntimeError('arm acceptance too late')
 deadline=time.monotonic()+duration+10
 while not result.done():
  spin();actual=gate();expected=sample(t,q,max(0,node.now()-start_at))
  if np.max(abs(actual-expected))>node.config['tracking_tolerance']:raise RuntimeError('arm tracking error')
  if time.monotonic()>deadline:raise RuntimeError('arm trial exceeded measurement window')
 value=result.result()
 if value.status!=4 or value.result.error_code!=0:raise RuntimeError('arm action failed '+value.result.error_string)
 handle=None;pending=None
 # Record fixed settling and sampling windows regardless of whether the final 5 deg criterion passes.
 begin=time.monotonic()
 while time.monotonic()-begin<2:spin();gate()
 begin=time.monotonic()
 while time.monotonic()-begin<5:
  spin();actual=gate()
  if np.max(abs(actual-target))>.15:raise RuntimeError('hold error exceeds tracking tolerance')
 finish=time.monotonic();values=np.array([x for tm,x in samples if begin<=tm<=finish]);error=np.rad2deg(values-target)
 eff=np.array([x for tm,x in efforts if begin<=tm<=finish]);row=dict(condition=label,samples=len(values),target_deg=np.rad2deg(target).tolist(),mean_deg=np.rad2deg(values.mean(axis=0)).tolist(),mean_error_deg=error.mean(axis=0).tolist(),mean_abs_error_deg=abs(error).mean(axis=0).tolist(),p95_abs_error_deg=np.percentile(abs(error),95,axis=0).tolist(),max_abs_error_deg=abs(error).max(axis=0).tolist(),std_deg=np.rad2deg(values.std(axis=0)).tolist(),all_joint_rmse_deg=float(np.sqrt(np.mean(error**2))),worst_error_deg=float(abs(error).max()),effort_max_abs=float(abs(eff).max()) if len(eff) else None)
 report['runs'].append(row);(out/'report.json').write_text(json.dumps(report,indent=2));print('RESULT '+json.dumps(row),flush=True)

try:
 previous=json.loads((out/'report.json').read_text()) if '--resume-after-on1' in sys.argv else None
 spin(3);gate();saved=params(keys);mode=params(control);
 if previous is not None:
  if previous['saved_parameters']!=saved or [r['condition'] for r in previous['runs']]!=['ON_1']:raise RuntimeError('resume state mismatch')
  report['runs']=previous['runs']
 report.update(saved_parameters=saved,control_parameters=mode,joints=list(ARM_NAMES));save()
 if mode['mode']!='gravity' or mode['contact_enabled'] or mode['bias_autozero'] or mode['autozero_now']:raise RuntimeError('gravity mode with no adaptive compensation required')
 run('EE_BASE_Z10')
except BaseException as exc:
 report['error']=str(exc);print('STOPPED '+str(exc),flush=True)
finally:
 if pending is not None and handle is None:
  def cancel_late(f):
   try:
    h=f.result()
    if h.accepted:h.cancel_goal_async()
   except Exception:pass
  pending.add_done_callback(cancel_late)
 if handle is not None and handle.accepted:
  f=handle.cancel_goal_async();end=time.monotonic()+3
  while not f.done() and time.monotonic()<end:spin()
  report['cancellation_confirmed']=f.done() and bool(f.result().goals_canceling)
 if review.ticket is not None:review.publish('closed',report['error'] or 'finished')
 if modified and saved is not None:
  try:set_params(saved);spin(.5);report['restored_parameters']=params(keys)==saved
  except Exception as exc:report['restore_error']=str(exc)
 save();can_socket.close();node.destroy_node();rclpy.shutdown()
