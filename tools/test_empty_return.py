import unittest
import numpy as np
from scipy.interpolate import BPoly
from test_fast_scan import FastScanTest
from lookup_empty_return import smooth_empty_return,VELOCITY_CAP,ACCELERATION_CAP,JERK_CAP
from lookup_fast_scan import compile_tail
from lookup_bt_support import prepare_plan

class EmptyReturnTest(unittest.TestCase):
 def test_geometry_and_continuous_acceleration_and_limits(self):
  FastScanTest.setUpClass();f=FastScanTest
  _,source,info=compile_tail(f.lib,'x+021_y-001_axis09_29',f.config,f.left)
  t=info['waypoint_at_s']+.51
  actual=np.r_[source.left_curve(t),source(t)]
  normal=prepare_plan(f.config,f.lib.ready,actual,velocity_cap=.45,acceleration_cap=.9)
  plan=smooth_empty_return(f.config,f.lib.ready,actual)
  np.testing.assert_allclose(normal['path'],plan['path'])
  self.assertGreater(plan['times'][-1],normal['times'][-1])
  idx=np.r_[0:7,14:21];rows=plan['path'][:,idx]
  curve=BPoly.from_derivatives(plan['times'],[[q,np.zeros(14),np.zeros(14)] for q in rows])
  dense=np.linspace(0,plan['times'][-1],10001)
  self.assertLessEqual(np.max(abs(curve(dense,1))),VELOCITY_CAP+1e-8)
  self.assertLessEqual(np.max(abs(curve(dense,2))),ACCELERATION_CAP+1e-8)
  self.assertLessEqual(np.max(abs(curve(dense,3))),JERK_CAP+1e-8)
  np.testing.assert_allclose(curve(plan['times'],1),0,atol=1e-10)
  np.testing.assert_allclose(curve(plan['times'],2),0,atol=1e-10)
  print('empty return seconds:',normal['times'][-1],'->',plan['times'][-1])
if __name__=='__main__':unittest.main()
