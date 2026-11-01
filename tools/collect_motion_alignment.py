"""Read-only acquisition. No action clients, publishers, FK or IK in callbacks."""
import json,time,threading,queue
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from deux_task_msgs.msg import ObjectDetectionArray,TaskStatus
from deux_hand_msgs.msg import LeaderGrip
from rosidl_runtime_py.convert import message_to_ordereddict
from deux_task_core.configuration import load_configuration
rclpy.init();node=rclpy.create_node('motion_alignment_audit_readonly')
root=Path('.setup/motion-alignment-audit');root.mkdir(exist_ok=True);out=root/str(time.time_ns());out.mkdir();(root/'latest.txt').write_text(str(out))
cfg,_=load_configuration('src/deux_task_bringup/config/installation.yaml');(out/'config.json').write_text(json.dumps(cfg,default=lambda x:x.tolist()))
q=queue.Queue(maxsize=10000);dropped=0;counts={};last_sample={};started=time.monotonic();active_seen=False;completed_at=None
stop=object()
def writer():
 with (out/'samples.jsonl').open('w',buffering=65536) as f:
  while True:
   item=q.get()
   if item is stop:break
   f.write(json.dumps(item)+'\n')
thread=threading.Thread(target=writer);thread.start()
def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9

def receive(kind,m):
 global dropped,active_seen,completed_at
 now=node.get_clock().now().nanoseconds/1e9
 if kind in ['controller','joints']:
  ts=stamp(m)
  if ts-last_sample.get(kind,-1)<.019:return
  last_sample[kind]=ts
 if kind=='status':
  if m.active:active_seen=True;completed_at=None
  elif active_seen:completed_at=completed_at or time.monotonic()
 counts[kind]=counts.get(kind,0)+1
 if kind=='controller':
  data=dict(stamp=stamp(m),names=list(m.joint_names),reference=list(m.reference.positions),feedback=list(m.feedback.positions),velocity=list(m.feedback.velocities))
 elif kind=='joints':data=dict(stamp=stamp(m),names=list(m.name),positions=list(m.position),velocity=list(m.velocity))
 elif kind in ['pose','prediction']:
  data=dict(stamp=stamp(m),objects=[dict(track=x.track_id,position=[x.pose.pose.position.x,x.pose.pose.position.y,x.pose.pose.position.z],velocity=[x.velocity.linear.x,x.velocity.linear.y,x.velocity.linear.z],valid=x.velocity_valid) for x in m.detections])
 else:data=message_to_ordereddict(m)
 try:q.put_nowait(dict(kind=kind,received=now,data=data))
 except queue.Full:dropped+=1
subs=[node.create_subscription(cls,topic,lambda m,k=k:receive(k,m),qos_profile_sensor_data if k!='status' else 10) for k,cls,topic in [('controller',JointTrajectoryControllerState,'/arm/deux_arm_controller/controller_state'),('joints',JointState,'/arm/joint_states'),('pose',ObjectDetectionArray,'/task/object_detections'),('prediction',ObjectDetectionArray,'/task/prediction_detections'),('status',TaskStatus,'/task/status'),('grip',LeaderGrip,'/hand/right/grip')]]
print(str(out),flush=True)
try:
 while time.monotonic()-started<600:
  rclpy.spin_once(node,timeout_sec=.1)
  if completed_at and time.monotonic()-completed_at>=4:break
except KeyboardInterrupt:pass
finally:
 q.put(stop);thread.join();(out/'capture.json').write_text(json.dumps(dict(counts=counts,dropped=dropped,active_seen=active_seen,duration_s=time.monotonic()-started),indent=2));node.destroy_node();rclpy.try_shutdown()
