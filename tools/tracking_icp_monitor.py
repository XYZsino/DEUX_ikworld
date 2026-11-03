"""Perception-only leases and throughput measurements. No robot action clients."""
import json,time
from copy import deepcopy
import yaml
from deux_task_core.lane_prediction import LanePrediction
from collections import deque
from pathlib import Path
from uuid import uuid4
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import String
from deux_task_msgs.msg import DetectionArray,ObjectDetectionArray,PerceptionRequest
from deux_task_runtime.common import ros_time
class Monitor(Node):
 def __init__(self):
  super().__init__('tracking_icp_monitor');self.target='';self.last_target='';self.last_seen=0.;self.lease_at=0.;self.req=None;self.frames=deque();self.track_frames=deque();self.results=deque();self.attempts=deque();self.latencies=deque();self.timings=deque();self.started=time.monotonic();self.accepted_total=0;self.attempt_total=0
  cfg=yaml.safe_load(Path('tracked_perception.thor.yaml').read_text())['perception_node']['ros__parameters'];self.lane=LanePrediction(cfg['table_plane_base'])
  self.lane_pub=self.create_publisher(ObjectDetectionArray,'/task/prediction_detections',qos_profile_sensor_data)
  self.retired=set();self.exit_reason=''
  self.create_subscription(String,'/task/tracking_event',self.tracking_event,10)
  self.log=Path('.setup/icp-live-test/events.jsonl');self.log.parent.mkdir(parents=True,exist_ok=True)
  self.requests=self.create_publisher(PerceptionRequest,'/task/perception_request',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));self.preview=self.create_publisher(String,'/task/icp_preview_status',10)
  self.create_subscription(DetectionArray,'/task/detections',self.detections,qos_profile_sensor_data);self.create_subscription(ObjectDetectionArray,'/task/object_detections',self.objects,qos_profile_sensor_data);self.create_subscription(String,'/task/icp_timing',self.timing,10);self.create_timer(.1,self.tick)
  self.lease('detect','')
 def tracking_event(self,msg):
  event=json.loads(msg.data);identity=event.get('target_id')
  if identity:self.retired.add(identity)
  if identity==self.target:
   self.target='';self.last_target='';self.exit_reason=event.get('reason','');self.lease('detect','')
 def now_s(self):return self.get_clock().now().nanoseconds/1e9
 def lease(self,mode,target):
  req=PerceptionRequest(request_id='preview-'+uuid4().hex,target_id=target,mode=mode);req.header.stamp=ros_time(self.now_s());req.expires_at=ros_time(self.now_s()+60);self.requests.publish(req);self.req=req;self.lease_at=time.monotonic()
 def detections(self,m):
  now=time.monotonic();self.frames.append(now);bags=[b for b in m.boxes if b.class_name=='bread_bag' and b.track_id not in self.retired]
  if self.target:
   if any(b.track_id==self.target for b in bags):self.last_seen=now;self.track_frames.append(now)
  elif len(bags)==1:
   self.target=self.last_target=bags[0].track_id;self.last_seen=now;self.track_frames.append(now);self.lease('locate',self.target)
 def objects(self,m):
  if self.req is None or m.request_id!=self.req.request_id:return
  if not any(d.track_id==self.target for d in m.detections):return
  output=deepcopy(m);diagnostics=[]
  stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  for d in output.detections:
   p=d.pose.pose.position;estimate=self.lane.update(d.track_id,stamp,[p.x,p.y,p.z])
   d.velocity_valid=estimate is not None;d.motion_covariance_valid=False
   if estimate:
    p.x,p.y,p.z=estimate['position'];d.velocity.linear.x,d.velocity.linear.y,d.velocity.linear.z=estimate['velocity'];d.velocity.angular.x=d.velocity.angular.y=d.velocity.angular.z=0.
    diagnostics.append(dict(track_id=d.track_id,**estimate))
  self.lane_pub.publish(output)
  with Path('.setup/icp-live-test/lane-predictions.jsonl').open('a') as f:f.write(json.dumps(dict(stamp=stamp,estimates=diagnostics))+'\n')
  now=time.monotonic();self.results.append(now);self.accepted_total+=1;self.latencies.append((now,(self.now_s()-(m.header.stamp.sec+m.header.stamp.nanosec/1e9))*1000))
 def timing(self,m):
  d=json.loads(m.data)
  if self.req is None or d.get('request_id')!=self.req.request_id:return
  now=time.monotonic();self.attempts.append(now);self.attempt_total+=1;self.timings.append((now,d))
  with self.log.open('a') as f:f.write(json.dumps(d)+'\n')
 def tick(self):
  now=time.monotonic()
  if self.target and now-self.last_seen>1.:
   self.target='';self.lease('detect','')
  elif now-self.lease_at>45:self.lease('locate' if self.target else 'detect',self.target)
  for queue in (self.frames,self.track_frames,self.results,self.attempts):
   while queue and queue[0]<now-10:queue.popleft()
  for queue in (self.latencies,self.timings):
   while queue and queue[0][0]<now-10:queue.popleft()
  window=min(10.,max(.1,now-self.started));values=[x[1] for x in self.latencies];workers=[x[1]['worker_ms'] for x in self.timings]
  data=dict(perception_only=True,target_id=self.target,exit_reason=self.exit_reason,tracking=bool(self.target),window_s=window,detection_hz=len(self.frames)/window,tracking_hz=len(self.track_frames)/window,icp_attempt_hz=len(self.attempts)/window,icp_pose_hz=len(self.results)/window,accepted_total=self.accepted_total,attempt_total=self.attempt_total,latency_p50_ms=float(np.median(values)) if values else None,latency_p95_ms=float(np.percentile(values,95)) if values else None,worker_p50_ms=float(np.median(workers)) if workers else None,last_error=next((x[1].get('error') for x in reversed(self.timings) if x[1].get('error')),None))
  self.preview.publish(String(data=json.dumps(data)));Path('.setup/icp-live-test/status.json').write_text(json.dumps(data,indent=2))
def main():
 rclpy.init();n=Monitor()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
