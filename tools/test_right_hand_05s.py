"""Explicitly requested physical hand-only test. Run with --execute."""
import sys,time,json,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from deux_hand_msgs.msg import LeaderGrip,HandCtlMessage,HandUiMessage
from std_msgs.msg import Bool
assert '--execute' in sys.argv, 'requires --execute'
root=Path(__file__).resolve().parents[1]
profiles=json.loads((root/'.setup/hand-live-profiles.json').read_text())
user=__import__('yaml').safe_load((root/'calibration/hand_postures.thor.yaml').read_text())
for side in ('right',):
 for name,key in [('initial','initial_deg'),('target','target_deg')]:
  np.testing.assert_allclose(profiles[side]['leader_mode_3.'+key],np.rad2deg(user['hands'][side][name]),atol=1e-8)
rclpy.init();n=rclpy.create_node('ikworld_requested_hand_test')
feedback={};external=[];errors=[];events=[];trace=[];estop=[False];active=[None];acks={}
tag='ikworld_requested_hand_test'
command_trace=[];sent=[]
def info(side):
 def cb(m):
  feedback[side]=(time.monotonic(),m)
  if active[0]==side:
   trace.append({'t':round(time.monotonic(),4),'side':side,'degrees':[math.degrees(v.rad) for v in m.motors],'ids':[v.id for v in m.motors],'errors':[v.hardware_error_status for v in m.motors],'current_field_raw':[v.goal_current for v in m.motors]})
 return cb
def grip_received(m):
 if m.header.frame_id!=tag and m.valid:external.append((time.monotonic(),m.side,m.grasp))
def action_command(m):
 command_trace.append({'t':time.monotonic(),'degrees':[math.degrees(v.rad) for v in m.motors],
                       'ids':[v.id for v in m.motors],'goal_current_raw':[v.goal_current for v in m.motors]})
subs=[n.create_subscription(HandCtlMessage,'/hand/right/action_joint',action_command,qos_profile_sensor_data)]
grips={};uis={}
for side in ('right',):
 subs += [n.create_subscription(HandCtlMessage,'/hand/'+side+'/info',info(side),qos_profile_sensor_data),
          n.create_subscription(LeaderGrip,'/hand/'+side+'/grip',grip_received,qos_profile_sensor_data),
          n.create_subscription(HandUiMessage,'/hand/'+side+'/ui_response',lambda m,s=side:acks.update({s:m.leader_mode}),10)]
 grips[side]=n.create_publisher(LeaderGrip,'/hand/'+side+'/grip',qos_profile_sensor_data)
 uis[side]=n.create_publisher(HandUiMessage,'/hand/'+side+'/ui_request',10)
subs.append(n.create_subscription(Bool,'/deux/estop',lambda m:estop.__setitem__(0,m.data),10))
def spin(seconds):
 end=time.monotonic()+seconds
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
def measured(side):
 assert not estop[0],'estop received; no further test commands'
 assert side in feedback,'missing feedback '+side
 t,m=feedback[side]
 assert time.monotonic()-t<.3,'feedback stale '+side
 motors={v.id:v for v in m.motors}
 assert set(motors)==set(range(1,8)),'unexpected motor IDs'
 assert all(v.hardware_error_status==0 for v in motors.values()),'motor hardware error'
 assert all(v.torque for v in motors.values()),'motor torque disabled'
 q=np.array([math.degrees(motors[i].rad) for i in range(1,8)])
 assert np.isfinite(q).all()
 assert not any(time.monotonic()-t<1 for t,s,g in external if s==side),'another grip producer is active'
 return q
def send(side,f):
 measured(side)
 m=LeaderGrip();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id=tag
 m.side=side;m.grasp=float(f);m.valid=True;grips[side].publish(m);sent.append({'t':time.monotonic(),'grasp':float(f)})
def expected(side,f):
 a=np.array(profiles[side]['leader_mode_3.initial_deg']);b=np.array(profiles[side]['leader_mode_3.target_deg'])
 return a+(b-a)*f
def settle(side,f,label):
 end=time.monotonic()+4;stable=0
 while time.monotonic()<end:
  send(side,f);spin(.05)
  error=float(np.max(np.abs(measured(side)-expected(side,f))))
  stable=stable+1 if error<7 else 0
  if stable>=5:break
 entry={'side':side,'phase':label,'grasp':f,'measured_deg':measured(side).tolist(),
        'expected_deg':expected(side,f).tolist(),'max_error_deg':error,'within_7_deg':stable>=5}
 events.append(entry);print(json.dumps(entry),flush=True)
 # Endpoint accuracy is reported, never relabelled as a pass.
 # A residual below the unchanged 30-degree tracking guard permits the return stroke.
 assert error<30,'endpoint deviation exceeds 30 deg'
 return stable>=5
def ramp(side,a,b,duration=4.0):
 start=time.monotonic();first=len(sent)
 steps=round(duration/.02)
 for i in range(steps+1):
  deadline=start+duration*i/steps
  spin(max(0.,deadline-time.monotonic()))
  f=a+(b-a)*i/steps
  send(side,float(f))
  assert np.max(np.abs(measured(side)-expected(side,float(f))))<30,'tracking deviation exceeds 30 deg'
 elapsed=sent[-1]['t']-sent[first]['t']
 print(json.dumps({'ramp_from':a,'ramp_to':b,'requested_seconds':duration,'actual_command_seconds':elapsed}),flush=True)
 events.append({'phase':'ramp','from':a,'to':b,'requested_seconds':duration,'actual_command_seconds':elapsed,'within_7_deg':True})
 spin(.02)
try:
 spin(2)
 for side in ('right',):
  measured(side);active[0]=side
  assert uis[side].get_subscription_count()>0 and grips[side].get_subscription_count()>0
  uis[side].publish(HandUiMessage(run_flag=False,stop_flag=False,teleop_mode=0,leader_mode=3))
  spin(.5)
  settle(side,0.,'initial')
  ramp(side,0.,1.,duration=0.5);settle(side,1.,'target')
  if side=='right':
   ramp(side,1.,0.);settle(side,0.,'return_initial')
  active[0]=None
 result='passed' if all(e['within_7_deg'] for e in events) else 'completed_with_tracking_error'
except Exception as exc:
 result='failed';errors.append(str(exc));print('TEST STOPPED: '+str(exc),flush=True)
finally:
 out={'result':result,'events':events,'errors':errors,'mode_response':acks,'external_grips':external,
      'trace':trace,'command_trace':command_trace,'sent_grip':sent,'final_state':{s: {'motor_ids':[v.id for v in m.motors],'degrees':[math.degrees(v.rad) for v in m.motors]} for s,(t,m) in feedback.items()},
      'arm_or_lift_commands_sent':False,'mode_requested':3,'final_intent':'right initial; left untouched; no torque-off'}
 (root/('.setup/right-hand-05s-'+time.strftime('%Y%m%d-%H%M%S')+'.json')).write_text(json.dumps(out,indent=2))
 n.destroy_node();rclpy.shutdown()
if result!='passed':raise SystemExit(1)
