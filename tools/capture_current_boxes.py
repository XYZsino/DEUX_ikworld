from pathlib import Path
import time,json
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from deux_task_perception.camera_input import CameraInput
from deux_task_perception.core import normalize_depth,normalize_rgb,stamp_ns
from deux_task_perception.icp_node import matrix
out=Path('.setup/box-plane-current');out.mkdir(exist_ok=True,parents=True)
rclpy.init();n=Node('capture_box_plane');tf=Buffer();listener=TransformListener(tf,n);frames=[];transforms=[];stamps=[];errors=[];info={}
def frame(f):
 try:bc=matrix(tf.lookup_transform('base_link',f.header.frame_id,Time.from_msg(f.header.stamp)))
 except Exception as e:errors.append(str(e));return
 if stamps and stamp_ns(f.header)<=stamps[-1]:return
 frames.append(normalize_depth(f.depth)[0].copy());transforms.append(bc);stamps.append(stamp_ns(f.header))
 if len(frames)==1:
  Image.fromarray(normalize_rgb(f.rgb)).save(out/'rgb.png');info['projection']=list(f.depth_info.p)
cam=CameraInput(n,frame);end=time.monotonic()+45
while len(frames)<150 and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
if len(frames)<100:raise RuntimeError('insufficient fresh frames '+str(len(frames))+str(errors[-1:]))
np.savez_compressed(out/'capture.npz',depths=np.array(frames),projection=info['projection'],base_from_camera=transforms[0],stamps=stamps)
report=dict(frames=len(frames),duration_s=(stamps[-1]-stamps[0])/1e9,tf_drift=float(np.max(abs(np.array(transforms)-transforms[0]))));(out/'capture.json').write_text(json.dumps(report));print(report)
n.destroy_node();rclpy.shutdown()
