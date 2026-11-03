"""Actual installed URDF and recent failed-grasp path; no hardware commands."""
import unittest,json,time
from pathlib import Path
from unittest.mock import patch
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from build_right_grasp_grid import Builder,ROOT
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_fast_scan import compile_tail
import lookup_soft_stop as stop

class JointwiseStopTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  b=Builder(ROOT/'src/deux_task_bringup/config/installation.yaml');cls.lib=ClassifiedGridLibrary(b.config,directory=ROOT/'artifacts/right_grasp_grid_classified',recommended=False,yaw_secondary=True,load_rotation_paths=False);cls.lib.ready[:7]=b.config['home_joints'][:7]
  left=json.loads((ROOT/'artifacts/dataset-barcode-pose/reading-pose.json').read_text())['left_joints_rad']
  cls.plan,_,_=compile_tail(cls.lib,'x+019_y+000_yaw-100',b.config,left_target=left,tail_duration=1.8,reuse_clearance=0.)
  cls.elapsed=7.025
 def test_reduces_ee_travel_with_continuous_start_and_all_joint_limits(self):
  started=time.monotonic();d=stop.build_soft_stop(self.plan,self.elapsed,self.lib.robot,self.lib.ready)
  self.assertEqual(d['profile'],'jointwise_min_ee_travel')
  curve=CubicHermiteSpline(self.plan['times_s'],self.plan['positions_rad'],self.plan['velocities_rad_s'])
  np.testing.assert_allclose(d['positions_rad'][0],curve(self.elapsed),atol=1e-10)
  np.testing.assert_allclose(d['velocities_rad_s'][0],curve(self.elapsed,1),atol=1e-10)
  np.testing.assert_allclose(d['velocities_rad_s'][-1],np.zeros(14),atol=1e-12)
  self.assertLessEqual(max(stop.dynamic_ratios(self.lib.robot,d)),1.000001)
  self.assertLess(d['ee_travel']['right_travel_m'],d['baseline_ee_travel']['right_travel_m'])
  self.assertLessEqual(d['ee_travel']['left_travel_m'],d['baseline_ee_travel']['left_travel_m']+1e-6)
  out=ROOT/'artifacts/jointwise-stop';out.mkdir(exist_ok=True)
  (out/'representative.json').write_text(json.dumps(dict(d,planning_wall_s=time.monotonic()-started),indent=2)+'\n')
  print(json.dumps({k:v for k,v in d.items() if k not in ('positions_rad','velocities_rad_s','times_s')},indent=2))
 def test_no_improvement_keeps_certified_original(self):
  with patch.object(stop,'travel',return_value=dict(left_travel_m=1.,right_travel_m=1.,left_max_displacement_m=1.,right_max_displacement_m=1.)):
   d=stop.build_soft_stop(self.plan,self.elapsed,self.lib.robot,self.lib.ready)
  self.assertEqual(d['profile'],'common_path_fallback')
 def test_cancellation_prevents_new_stop(self):
  with self.assertRaises(InterruptedError):stop.build_soft_stop(self.plan,self.elapsed,self.lib.robot,self.lib.ready,lambda:True)

if __name__=='__main__':unittest.main()
