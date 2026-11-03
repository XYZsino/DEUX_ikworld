import ast,unittest,time
from pathlib import Path
from types import SimpleNamespace as S
import numpy as np
from via_depth_gate import box_depth
from scan_bbox_gate import matching_boxes

class ViaDepthTests(unittest.TestCase):
 def test_depth_band_background_invalid_and_units(self):
  depth=np.full((100,100),1.,float)
  self.assertTrue(box_depth(depth,[10,10,90,90],1.1)['depth_passed'])
  self.assertFalse(box_depth(depth,[10,10,90,90],.6)['depth_passed'])
  self.assertTrue(box_depth(depth*1000,[10,10,90,90],1.,scale=.001)['depth_passed'])
  depth[:]=np.nan
  with self.assertRaises(ValueError):box_depth(depth,[10,10,90,90],1.)
  depth[:]=0
  with self.assertRaises(ValueError):box_depth(depth,[10,10,90,90],1.)
  with self.assertRaises(ValueError):box_depth(np.ones((100,100)),[10,10,90,90],-1.)
 def test_counts_distinct_valid_images_once_and_rejects_depth_background(self):
  source=ast.parse(Path(__file__).with_name('lookup_bt_node.py').read_text())
  cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='LookupTaskNode')
  method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='receive_scan_boxes')
  scope=dict(time=time,seconds=lambda t:t.sec+t.nanosec/1e9,matching_boxes=matching_boxes)
  exec(compile(ast.Module(body=[method],type_ignores=[]),'receive','exec'),scope)
  roi=dict(width=640,height=360,points=[[0,0],[640,0],[640,360],[0,360]])
  ev=dict(min_confidence=.5,request_id='current',arrival_s=10.,roi=roi,bag_min_area_px2=1000.,barcode_min_area_px2=300.,fresh_frames=0,matched_frames=0,matched_stamps_s=[])
  depth=S(filter=lambda m,rows,t:(rows,[]))
  node=S(scan_bbox_context=ev,scan_bbox_evidence=ev,scan_seen_frames=set(),scan_camera_info=S(width=640,height=360,header=S(frame_id='rgb')),config=dict(max_object_age=.5),now=lambda:10.4,via_depth=depth,get_parameter=lambda _:S(value=.15))
  def msg(i):
   b=S(class_name='bread_bag',xyxy=[100.,100.,150.,150.],track_id='bag',confidence=.9)
   return S(header=S(frame_id='rgb',stamp=S(sec=10,nanosec=i*10000000)),request_id='current',boxes=[b,b])
  for i in range(1,5):
   scope['receive_scan_boxes'](node,msg(i));scope['receive_scan_boxes'](node,msg(i))
  self.assertEqual(ev['matched_frames'],4)
  depth.filter=lambda m,rows,t:([],[dict(depth_passed=False)])
  scope['receive_scan_boxes'](node,msg(5));self.assertEqual(ev['matched_frames'],4)
  depth.filter=lambda m,rows,t:(rows,[])
  scope['receive_scan_boxes'](node,msg(6));self.assertEqual(ev['matched_frames'],5)
  ev['matched_frames']=0;ev['deadline_monotonic']=time.monotonic()-1
  scope['receive_scan_boxes'](node,msg(7));self.assertEqual(ev['matched_frames'],0)
if __name__=='__main__':unittest.main()
