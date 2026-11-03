import json,tempfile,unittest,threading
from unittest.mock import patch
from pathlib import Path
import numpy as np
from PIL import Image
from via_depth_gate import box_depth
from via_frame_archive import FrameArchive

class HandExclusionTest(unittest.TestCase):
 def test_foreground_hand_must_not_validate_background_bag(self):
  d=np.full((100,100),.9);hand=np.zeros((100,100),bool);hand[15:85,15:85]=True;d[hand]=.39
  self.assertTrue(box_depth(d,[0,0,100,100],.39)['depth_passed'])
  result=box_depth(d,[0,0,100,100],.39,excluded=hand)
  self.assertFalse(result['depth_passed']);self.assertAlmostEqual(result['box_depth_m'],.9)
 def test_real_bag_outside_hand_remains_valid(self):
  d=np.full((100,100),.42);hand=np.zeros((100,100),bool);hand[:,:45]=True
  self.assertTrue(box_depth(d,[0,0,100,100],.39,excluded=hand)['depth_passed'])
  with self.assertRaises(ValueError):box_depth(d,[0,0,100,100],.39,excluded=np.ones_like(hand))
 def test_archive_roundtrip_and_metadata(self):
  with tempfile.TemporaryDirectory() as tmp:
   a=FrameArchive(tmp);rgb=np.full((20,30,3),100,np.uint8);depth=np.full((20,30),.4,np.float32);depth[0,0]=np.nan;mask=np.zeros((20,30),bool);mask[:,0]=True
   folder=Path(a.save('test',12,rgb,depth,mask,dict(accepted=False)));a.close()
   self.assertFalse(list(folder.glob('*.jpg')));self.assertFalse(list(folder.glob('*.jpeg')))
   np.testing.assert_allclose(np.load(folder/'depth_m.npy'),depth,equal_nan=True)
   self.assertEqual(json.loads((folder/'decision.json').read_text())['accepted'],False)
   self.assertTrue((folder/'hand_excluded.png').exists());self.assertFalse((folder/'overlay.jpg').exists())
 def test_archive_slow_writer_is_nonblocking_and_snapshots_inputs(self):
  with tempfile.TemporaryDirectory() as tmp:
   entered=threading.Event();release=threading.Event();original=np.save
   def slow(image,*args,**kwargs):
    entered.set();release.wait(3.);return original(image,*args,**kwargs)
   a=FrameArchive(tmp);rgb=np.full((20,30,3),100,np.uint8);depth=np.full((20,30),.4,np.float32);metadata=dict(accepted=True)
   try:
    with patch('via_frame_archive.np.save',slow):
     folder=Path(a.save('slow',1,rgb,depth,None,metadata))
     self.assertTrue(entered.wait(1.));self.assertFalse(a.ready());self.assertEqual(a.status()['pending'],1)
     rgb[:]=0;depth[:]=0;metadata['accepted']=False
     release.set();a.close()
    self.assertEqual(a.status()['saved'],1)
    self.assertFalse(list(folder.glob('*.jpg')))
    self.assertAlmostEqual(float(np.load(folder/'depth_m.npy').mean()),.4,places=5)
    self.assertTrue(json.loads((folder/'decision.json').read_text())['accepted'])
   finally:release.set();a.close()
 def test_storage_error_is_reported_without_poisoning_enqueue(self):
  with tempfile.TemporaryDirectory() as tmp:
   errors=[];a=FrameArchive(tmp,error_callback=errors.append)
   with patch('via_frame_archive.shutil.disk_usage',side_effect=OSError('synthetic disk error')):
    a.save('failure',1,np.zeros((2,2,3),np.uint8),np.zeros((2,2),np.float32),None,{})
    a.close()
   self.assertEqual(a.status()['failed'],1);self.assertTrue(errors);self.assertTrue(a.ready())
if __name__=='__main__':unittest.main()
