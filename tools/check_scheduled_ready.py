import sys,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
sys.path.insert(0,'tools')
from ready_recovery import compute_ready
from deux_task_core.model import ARM_NAMES
p=json.loads(Path('.setup/moving-upper-trial/latest-intercept-input.json').read_text());c=p['config'];candidate=json.loads(Path('.setup/scheduled-standby-candidates.json').read_text())['candidates'][0]
rclpy.init();node=rclpy.create_node('scheduled_ready_readonly_check');messages=[]
sub=node.create_subscription(JointState,'/arm/joint_states',messages.append,qos_profile_sensor_data);end=time.monotonic()+5
while not messages and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
if not messages:raise RuntimeError('no measured arm joints')
m=messages[-1];values=dict(zip(m.name,m.position));q=np.array(c['home_joints']);q[:7]=[values[n] for n in ARM_NAMES[:7]];q[14:21]=[values[n] for n in ARM_NAMES[7:]];c['_stationary_left_arm']=q[:7].tolist();goal=q.copy();goal[14:21]=candidate['right_arm_rad']
result=compute_ready(c,q,goal);result.update(current=q.tolist(),goal=goal.tolist(),source_stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9)
Path('.setup/scheduled-ready-route.json').write_text(json.dumps(result,indent=2));print('validated geometric ready route',len(result['right_waypoints']),'waypoints')
node.destroy_node();rclpy.shutdown()
