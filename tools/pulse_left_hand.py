"""Explicit hand-only repeat of the configured AlignBarcode scalar pulse."""
import json, time, sys
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from deux_hand_msgs.msg import LeaderGrip, HandCtlMessage, HandUiMessage
from deux_task_msgs.msg import TaskStatus
from std_msgs.msg import Bool
from deux_task_runtime.common import LATCHED
from deux_task_runtime.align_hand_pulse import pulse_fraction
import argparse
parser=argparse.ArgumentParser()
parser.add_argument('--execute',action='store_true')
parser.add_argument('--opening',type=float,default=.3)
parser.add_argument('--target',type=float,default=None)
parser.add_argument('--phase-duration',type=float,default=.5)
args=parser.parse_args()
assert args.execute
assert args.target is None or 0 <= args.target <= 1
pulse_fraction(0,args.opening,args.phase_duration)
rclpy.init(); n=rclpy.create_node('requested_left_hand_pulse')
tag='requested_left_hand_pulse'; state={}; external=[]; trace=[]; commands=[]; estop=[False]
def save(key,m):state[key]=(time.monotonic(),m)
def receive_grip(m):
    if m.valid and m.header.frame_id!=tag:external.append(time.monotonic())
def receive_hand(m):
    save('hand',m)
    trace.append({'t':time.monotonic(),'rad':[v.rad for v in m.motors]})
subs=[n.create_subscription(HandCtlMessage,'/hand/left/info',receive_hand,qos_profile_sensor_data),
      n.create_subscription(HandUiMessage,'/hand/left/ui_response',lambda m:save('mode',m),10),
      n.create_subscription(TaskStatus,'/task/status',lambda m:save('task',m),LATCHED),
      n.create_subscription(LeaderGrip,'/hand/left/grip',receive_grip,qos_profile_sensor_data),
      n.create_subscription(Bool,'/deux/estop',lambda m:estop.__setitem__(0,m.data),10)]
pub=n.create_publisher(LeaderGrip,'/hand/left/grip',qos_profile_sensor_data)
def guard():
    now=time.monotonic()
    assert not estop[0],'e-stop'
    assert 'task' in state and not state['task'][1].active,'BT active or unknown'
    assert 'mode' in state and now-state['mode'][0]<2,'mode unavailable/stale'
    m=state['mode'][1]
    assert m.leader_mode==3 and m.teleop_mode==0 and m.run_flag and not m.stop_flag,'requires mode 3 / RUN / teleop off'
    assert 'hand' in state and now-state['hand'][0]<.2,'hand feedback unavailable/stale'
    motors=state['hand'][1].motors
    assert {v.id for v in motors}==set(range(1,8)),'motor IDs'
    assert all(v.torque and v.hardware_error_status==0 for v in motors),'motor error or torque off'
    assert not external or now-external[-1]>1,'competing grip commands'
    assert n.count_publishers('/hand/left/cmd')==0,'competing direct command publisher'
    assert pub.get_subscription_count()>0,'driver unavailable'
try:
    end=time.monotonic()+2
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
    guard();start=time.monotonic()
    while True:
        rclpy.spin_once(n,timeout_sec=.01);guard()
        elapsed=time.monotonic()-start
        if args.target is None:
            f,done=pulse_fraction(elapsed,args.opening,args.phase_duration)
        else:
            f=args.target
            done=elapsed>=args.phase_duration
        m=LeaderGrip();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id=tag
        m.side='left';m.grasp=f;m.valid=True;pub.publish(m)
        commands.append({'elapsed':elapsed,'grasp':f})
        if done:break
    result='command_sequence_complete'
finally:
    out={'result':locals().get('result','stopped'),'commands':commands,'feedback':trace,'arm_commands_sent':False}
    Path('.setup/left-hand-pulse.json').write_text(json.dumps(out,indent=2))
    print(json.dumps({k:v for k,v in out.items() if k not in ('commands','feedback')}),flush=True)
    n.destroy_node();rclpy.shutdown()
