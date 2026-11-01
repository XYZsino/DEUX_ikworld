"""IK/timing preview; subscribes measurements and publishes JSON only."""
import json,time,multiprocessing
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState,CameraInfo
from std_msgs.msg import String
from deux_task_msgs.msg import ObjectDetectionArray
from deux_task_core.configuration import load_configuration
from deux_task_core.model import ARM_NAMES,JOINT_NAMES
from deux_task_core.position_first_preview import warm,solve
from deux_task_perception.tracking_roi import read_roi,default_path
from deux_task_core.prediction_audit import PredictionAudit
from deux_task_perception.perception_core import transform_matrix
class Preview(Node):
 def __init__(self):
  super().__init__('grasp_timing_preview');self.config,_=load_configuration('src/deux_task_bringup/config/installation.yaml');self.config['home_joints']=json.loads(Path('.setup/selected-ready.json').read_text())['chosen']['new_home'];self.config['preview_min_grasp_yaw_deg']=30.;self.camera=None;self.calibration_valid=False;self.arm=None;self.objects=None;self.last_submit=0.;self.job=None;self.result=dict(feasible=False,reason='IK 초기화 중');self.pool=ProcessPoolExecutor(1,mp_context=multiprocessing.get_context('spawn'));self.future=self.pool.submit(warm,self.config);self.warming=True
  self.audit=PredictionAudit();self.attempts=0;self.successes=0;self.audit_last=None;self.audit_matched=0
  self.create_subscription(ObjectDetectionArray,'/task/object_detections',self.audit_observation,qos_profile_sensor_data)
  self.tracking_status={}
  self.create_subscription(String,'/task/icp_preview_status',self.on_tracking_status,10)
  self.create_subscription(CameraInfo,'/zed/color/camera_info',lambda m:setattr(self,'camera',m),qos_profile_sensor_data)
  self.create_subscription(String,'/task/camera_calibration_status',lambda m:setattr(self,'calibration_valid',bool(json.loads(m.data).get('valid'))),10)
  self.pub=self.create_publisher(String,'/task/grasp_timing_preview',10)
  self.create_subscription(JointState,'/arm/joint_states',lambda m:setattr(self,'arm',m),qos_profile_sensor_data)
  self.create_subscription(ObjectDetectionArray,'/task/prediction_detections',lambda m:setattr(self,'objects',m),qos_profile_sensor_data)
  self.create_timer(.2,self.tick)
 def on_tracking_status(self,msg):
  self.tracking_status=json.loads(msg.data)
 def audit_observation(self,msg):
  for d in msg.detections:
   p=d.pose.pose.position
   for entry in self.audit.observe(d.track_id,self.stamp(msg.header),[p.x,p.y,p.z]):
    if entry['matched']:self.audit_matched+=1;self.audit_last=entry
    with Path('.setup/icp-live-test/live-continuous-audit.jsonl').open('a') as f:f.write(json.dumps(entry)+'\n')
 def now_s(self):return self.get_clock().now().nanoseconds/1e9
 @staticmethod
 def stamp(h):return h.stamp.sec+h.stamp.nanosec/1e9
 def tick(self):
  now=self.now_s()
  if self.future is not None and self.future.done():
   try:
    value=self.future.result()
    if self.warming:self.result=dict(feasible=False,reason='유효 ICP 속도·관절 피드백 대기');self.warming=False
    else:
     value.update(target_id=self.job['target_id'],computed_at_s=now)
     if not self.tracking_status.get('tracking') or self.tracking_status.get('target_id')!=self.job['target_id']:value.update(feasible=False,reason='ROI 추적 종료/대상 변경')
     if self.objects is None or not any(d.track_id==self.job['target_id'] for d in self.objects.detections):value.update(feasible=False,reason='추적 대상 변경')
     if self.arm is None or now-self.stamp(self.arm.header)>self.config['max_joint_age']:value.update(feasible=False,reason='관절 피드백 지연')
     elif value.get('feasible'):
      measured=dict(zip(self.arm.name,self.arm.position));q=np.array(value['joint_start']);indices=[JOINT_NAMES.index(n) for n in ARM_NAMES]
      if max(abs(measured[n]-q[i]) for n,i in zip(ARM_NAMES,indices))>self.config['start_tolerance']:value.update(feasible=False,reason='계획 중 관절 자세 변경')
     self.attempts+=1
     if value.get('feasible'):
      self.successes+=1
      if value.get('preview_kind')!='position_first_alignment':self.audit.add(value)
     with Path('.setup/icp-live-test/live-continuous-results.jsonl').open('a') as f:f.write(json.dumps(value)+'\n')
     self.result=value
     Path('.setup/icp-live-test/grasp-preview.json').write_text(json.dumps(value,indent=2))
   except Exception as e:self.result=dict(feasible=False,reason=str(e));self.warming=False
   self.future=None
  if self.future is None and time.monotonic()-self.last_submit>=1. and self.arm is not None and self.objects is not None and self.camera is not None and self.calibration_valid:
   age=now-self.stamp(self.objects.header)
   if len(self.arm.velocity)!=len(self.arm.name) or any(abs(v)>.05 for n,v in zip(self.arm.name,self.arm.velocity) if n in ARM_NAMES):
    self.result=dict(feasible=False,preview_kind='position_first_alignment',reason='위치 우선 계획: 정지 관절 피드백 대기');self.pub.publish(String(data=json.dumps(dict(self.result,preview_only=True,attempts=self.attempts,successes=self.successes))));return
   choices=[d for d in self.objects.detections if d.velocity_valid and self.tracking_status.get('tracking') and d.track_id==self.tracking_status.get('target_id')]
   if 0<=age<=.75 and 0<=now-self.stamp(self.arm.header)<=self.config['max_joint_age'] and len(choices)==1:
    d=choices[0];values=dict(zip(self.arm.name,self.arm.position))
    if all(n in values for n in ARM_NAMES):
     q=np.array(self.config['home_joints']);q[7:14]=self.config['hand_closed'][:7];q[21:]=self.config['hand_open'][7:]
     for n in ARM_NAMES:q[JOINT_NAMES.index(n)]=values[n]
     p=d.pose.pose.position;r=d.pose.pose.orientation;obj=transform_matrix([p.x,p.y,p.z],[r.x,r.y,r.z,r.w]);v=d.velocity
     Path('.setup/icp-live-test/grasp-preview-input.json').write_text(json.dumps(dict(q=q.tolist(),object_pose=obj.tolist(),velocity=[v.linear.x,v.linear.y,v.linear.z,v.angular.z],age=age,acquired_s=self.stamp(self.objects.header))))
     geometry=dict(roi=read_roi(default_path()),camera=json.loads(Path('calibration/zed2i_extrinsics.thor.json').read_text())['T_base_camera_optical'],projection=list(self.camera.p),width=self.camera.width,height=self.camera.height)
     self.job=dict(target_id=d.track_id);self.last_submit=time.monotonic();self.future=self.pool.submit(solve,self.config,q.tolist(),obj.tolist(),[v.linear.x,v.linear.y,v.linear.z,v.angular.z],age,self.last_submit,self.stamp(self.objects.header),geometry)
  msg=dict(self.result,attempts=self.attempts,successes=self.successes,audit_matched=self.audit_matched,audit_last=self.audit_last,planning=self.future is not None,preview_only=True)
  if msg.get('feasible'):
   if self.objects is None or now-self.stamp(self.objects.header)>.75 or not any(d.track_id==msg.get('target_id') for d in self.objects.detections):msg.update(feasible=False,reason='현재 대상 ICP 관측 지연/변경')
   elif self.arm is None or not 0<=now-self.stamp(self.arm.header)<=self.config['max_joint_age']:msg.update(feasible=False,reason='관절 피드백 지연')
  if not msg.get('feasible') and self.future is None:
   if self.arm is None:msg['reason']='팔 관절 피드백 대기'
   elif not 0<=now-self.stamp(self.arm.header)<=self.config['max_joint_age']:msg['reason']='관절 피드백 지연'
   elif self.objects is None or now-self.stamp(self.objects.header)>.75:msg['reason']='최신 ICP 포즈 대기'

  if not self.calibration_valid:msg.update(feasible=False,reason='카메라 보정 상태 대기')
  if not self.tracking_status.get('tracking') or self.tracking_status.get('target_id')!=msg.get('target_id'):msg.update(feasible=False,reason='ROI 진입 대상 대기')
  if msg.get('feasible'):msg['contact_in_s']=msg['contact_at_s']-now;msg['expired']=msg['contact_in_s']<=0
  self.pub.publish(String(data=json.dumps(msg)))
 def destroy_node(self):
  self.pool.shutdown(wait=False,cancel_futures=True);return super().destroy_node()
def main():
 rclpy.init();n=Preview()
 try:rclpy.spin(n)
 finally:
  n.destroy_node()
  if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
