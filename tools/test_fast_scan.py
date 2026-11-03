"""Preserved incoming curve and audited scan-tail timing, offline only."""
import json,unittest
import numpy as np,yaml
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_lift_scan import compile_path,VIA
from lookup_fast_scan import ROOT,PROFILE,compile_tail,load_audited

class FastScanTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  p=ROOT/'src/deux_task_bringup/config/installation.yaml';cls.config=yaml.safe_load(p.read_text());cls.config['robot']['urdf_path']=str((p.parent/cls.config['robot']['urdf_path']).resolve())
  cls.lib=ClassifiedGridLibrary(cls.config,directory=ROOT/'artifacts/right_grasp_grid_classified',recommended=False,yaw_secondary=True,load_rotation_paths=False)
  cls.lib.ready[:7]=cls.config['home_joints'][:7]
  cls.left=json.loads((ROOT/'artifacts/dataset-barcode-pose/reading-pose.json').read_text())['left_joints_rad']
 def test_reference_preserves_right_prefix_and_shortens_only_tail(self):
  key='x+021_y-001_axis09_29'
  old,original,old_info=compile_path(self.lib,key,self.config,left_target=self.left)
  plan,curve,info=compile_tail(self.lib,key,self.config,self.left)
  via=old_info['waypoint_at_s']
  self.assertEqual(info['waypoint_at_s'],via);self.assertEqual(info['duration_s'],7.8)
  for derivative in (0,1,2):
   np.testing.assert_allclose(curve(np.linspace(0,via,151),derivative),original(np.linspace(0,via,151),derivative),atol=1e-12)
   np.testing.assert_allclose(curve.tail(0,derivative),original(via,derivative),atol=1e-10)
  np.testing.assert_allclose(curve(via),VIA,atol=1e-10)
  np.testing.assert_allclose(curve.left_curve(np.linspace(0,via,31)),np.tile(self.lib.ready[:7],(31,1)))
  np.testing.assert_allclose(curve.left_curve(info['duration_s']),self.left,atol=1e-10)
  np.testing.assert_allclose(curve(info['duration_s']),self.config['scan_right_joints'],atol=1e-10)
  self.assertEqual(plan['hand_schedule'],old['hand_schedule']);self.assertGreater(info['minimum_conveyor_clearance_m'],0)
  oldt=np.asarray(old['times_s']);newt=np.asarray(plan['times_s']);mask=oldt<=via
  np.testing.assert_allclose(newt[newt<=via],oldt[mask])
  np.testing.assert_allclose(np.asarray(plan['positions_rad'])[newt<=via,7:],np.asarray(old['positions_rad'])[mask,7:],atol=1e-12)
 def test_valid_cache_loads_all_accepted_cells(self):
  paths,rejected=load_audited(self.lib,self.config,self.left)
  self.assertEqual(len(paths),2175);self.assertEqual(len(rejected),2)
 def test_cache_rejects_changed_left_target(self):
  with self.assertRaisesRegex(ValueError,'inputs changed'):load_audited(self.lib,self.config,np.asarray(self.left)+.01)
 def test_returned_plan_uses_no_slower_than_legacy_tail(self):
  report=json.loads((ROOT/'artifacts/scan-speed-search/library-validation.json').read_text())
  self.assertEqual(report['profile'],PROFILE)
  self.assertEqual(report['total'],len(self.lib.entries))
  self.assertEqual(len(report['accepted'])+len(report['rejected']),len(self.lib.entries))
  for row in report['accepted'].values():
   self.assertLessEqual(row['duration_s'],row['original_duration_s']+1e-8)
if __name__=='__main__':unittest.main()
