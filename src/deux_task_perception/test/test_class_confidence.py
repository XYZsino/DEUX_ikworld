from types import SimpleNamespace
import numpy as np
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from deux_task_msgs.msg import Observation
from deux_task_perception.tracked_detection_node import TrackedDetectionNode

def test_detection_filters_bags_before_single_slot_but_keeps_lower_confidence_barcode():
    h=Header(stamp=Time(sec=1),frame_id='camera')
    info=CameraInfo(header=h,width=8,height=8,k=[10.,0.,4.,0.,10.,4.,0.,0.,1.],p=[10.,0.,4.,0.,0.,10.,4.,0.,0.,0.,1.,0.],r=np.eye(3).ravel().tolist())
    o=Observation(header=h,sequence=1,rgb=Image(header=h,width=8,height=8,encoding='rgb8',step=24,data=bytes(192)),depth=Image(header=h,width=8,height=8,encoding='32FC1',step=32,data=np.ones((8,8),np.float32).tobytes()),rgb_info=info,depth_info=info)
    class Detector:
        identity='test'
        def __call__(self,rgb):
            return [(0,'s1',.94,[0.,0.,7.,7.]),(0,'s1',.85,[1.,1.,3.,3.]),(1,'bar',.6,[2.,2.,4.,4.]),(1,'bar',.49,[4.,4.,6.,6.])]
    node=SimpleNamespace(detector=Detector(),barcode_confidence=.5,cfg=dict(width=8,height=8,sync_slop_s=.02,ignore_tracking_ids=True,input_bag_classes='s1,s1_1',input_barcode_class='bar',bag_class='bread_bag',barcode_class='barcode',confidence=.9))
    out=TrackedDetectionNode.detect(node,o)
    assert [(b.class_name,b.confidence) for b in out.boxes]==[('bread_bag',.94),('barcode',.6)]
    assert out.boxes[0].track_id=='single-bag'
