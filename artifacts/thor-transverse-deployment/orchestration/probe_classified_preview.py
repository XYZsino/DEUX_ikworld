import json,time
from collections import Counter
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.action.graph import get_action_client_names_and_types_by_node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from deux_task_msgs.msg import ObjectDetectionArray
rclpy.init();node=Node('rotation_preview_readonly_probe');counts=Counter();latest={}
def receive(name):
 def call(msg):
  counts[name]+=1;latest[name]=json.loads(msg.data)
 return call
for topic,name in [('/task/grid_grasp/preview','preview'),('/task/camera_calibration_status','calibration'),
                   ('/task/icp_preview_status','tracking')]:node.create_subscription(String,topic,receive(name),10)
node.create_subscription(JointState,'/arm/joint_states',lambda m:counts.update(['joints']),qos_profile_sensor_data)
def detection(m):
 counts['predictions']+=1;latest['detected_objects']=len(m.detections)
node.create_subscription(ObjectDetectionArray,'/task/prediction_detections',detection,qos_profile_sensor_data)
try:
 deadline=time.monotonic()+12
 while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
 p=latest.get('preview',{})
 assert counts['preview']>0 and p.get('preview_only') and not p.get('execution_enabled'),latest
 assert not p.get('motion_requested') and p.get('cells')==2177 and p.get('contact_cells')==146 and p.get('normal_contact_cells')==118 and p.get('transverse_contact_cells')==74,p
 assert counts['joints']>0 and latest.get('calibration',{}).get('valid'),latest
 publishers={topic:[info.node_name for info in node.get_publishers_info_by_topic(topic) if info.node_name=='grid_grasp_trial']
             for topic in ['/arm/deux_arm_controller/joint_trajectory','/hand/right/grip','/hand/right/cmd']}
 clients=get_action_client_names_and_types_by_node(node,'grid_grasp_trial','/')
 assert not any(publishers.values()) and not clients,(publishers,clients)
 report=dict(actual_hardware_streams=True,counts=dict(counts),latest=latest,
             grid_command_publishers=publishers,grid_action_clients=clients,
             action_requested=False,arm_or_hand_commands_issued=False,verified_at=time.time())
 out=Path('/home/xyz_thor/DEUX_ikworld/.setup/grid-grasp-classified-test/live-probe.json')
 out.write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
finally:node.destroy_node();rclpy.shutdown()
