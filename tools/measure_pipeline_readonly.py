import json,time
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from deux_task_msgs.msg import DetectionArray,ObjectDetectionArray
from std_msgs.msg import String
rclpy.init();n=rclpy.create_node('pipeline_timing_readonly');out=Path('.setup/pipeline-timing');out.mkdir(exist_ok=True);f=(out/'live.jsonl').open('w',buffering=1)
def rec(kind,m):
 now=n.get_clock().now().nanoseconds/1e9
 if kind=='icp':d=dict(kind=kind,received=now,data=json.loads(m.data))
 else:
  stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  items=m.boxes if kind=='detection' else m.detections
  d=dict(kind=kind,received=now,stamp=stamp,age_ms=1000*(now-stamp),tracks=[x.track_id for x in items],valid=sum(getattr(x,'velocity_valid',False) for x in items))
 f.write(json.dumps(d)+'\n')
subs=[n.create_subscription(cls,topic,lambda m,k=k:rec(k,m),qos_profile_sensor_data if k!='icp' else 10) for k,cls,topic in [('detection',DetectionArray,'/task/detections'),('pose',ObjectDetectionArray,'/task/object_detections'),('prediction',ObjectDetectionArray,'/task/prediction_detections'),('icp',String,'/task/icp_timing')]]
end=time.monotonic()+90
try:
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
finally:f.close();n.destroy_node();rclpy.try_shutdown()
