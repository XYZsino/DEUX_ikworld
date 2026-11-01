"""Read-only observations; no command publishers or action clients."""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from deux_task_msgs.msg import ObjectDetectionArray
rclpy.init()
n=Node('conveyor_spline_readonly_validation')
out=Path('.setup/conveyor-spline-ready/live.jsonl')
out.parent.mkdir(parents=True,exist_ok=True)
f=out.open('a',buffering=1)
def receive(m,kind):
 t=m.header.stamp.sec+m.header.stamp.nanosec/1e9
 for d in m.detections:
  p=d.pose.pose.position;v=d.velocity.linear
  f.write(json.dumps(dict(kind=kind,stamp=t,received=time.time(),track=d.track_id,position=[p.x,p.y,p.z],velocity=[v.x,v.y,v.z],velocity_valid=d.velocity_valid))+'\n')
subs=[n.create_subscription(ObjectDetectionArray,topic,lambda m,k=k:receive(m,k),qos_profile_sensor_data) for k,topic in [('raw','/task/object_detections'),('prediction','/task/prediction_detections')]]
end=time.monotonic()+180
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
f.close();n.destroy_node();rclpy.shutdown()
