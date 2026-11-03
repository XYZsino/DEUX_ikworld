import unittest
from types import SimpleNamespace as S
import numpy as np
from deux_task_runtime.reader_yaw import rotate_target,adjusted_reader
class ReaderYawTest(unittest.TestCase):
 def test_extra_right_yaw_preserves_origin_and_changes_axis(self):
  target=np.eye(4);target[:3,3]=[.4,.1,1.2];target[:3,:3]=np.array([[0.,0.,1.],[1.,0.,0.],[0.,1.,0.]])
  out=rotate_target(target,-15.)
  np.testing.assert_array_equal(out[:3,3],target[:3,3]);self.assertAlmostEqual(np.degrees(np.arctan2(out[1,2],out[0,2])),-15.)
  np.testing.assert_allclose(out[:3,:3].T@out[:3,:3],np.eye(3),atol=1e-12)
 def test_only_reader_target_adjusted_and_backend_restored_after_failure(self):
  calls=[]
  class Solver:
   def solve_reader_axis(self,*args):calls.append(args);return args[-1]
  original=Solver();planner=S(config={'barcode_position_only':True},robot=S(ik_solver=original));target=np.eye(4);offset=np.eye(4);seed=np.zeros(28)
  with self.assertRaises(RuntimeError):
   with adjusted_reader(planner,'AlignBarcode',-15.):
    planner.robot.ik_solver.solve_reader_axis('left',target,offset,seed);raise RuntimeError('IK failed')
  self.assertIs(planner.robot.ik_solver,original);np.testing.assert_allclose(calls[0][1],rotate_target(target,-15.));self.assertIs(calls[0][2],offset)
  with adjusted_reader(planner,'ScanPose',-15.):self.assertIs(planner.robot.ik_solver,original)
 def test_invalid_angle_and_unsupported_backend_rejected(self):
  for value in [float('nan'),46.,-46.]:
   with self.assertRaises(ValueError):rotate_target(np.eye(4),value)
  with self.assertRaises(ValueError):
   with adjusted_reader(S(config={'barcode_position_only':True},robot=S()),'AlignBarcode',-15.):pass
if __name__=='__main__':unittest.main()
