import unittest,json,yaml
from pathlib import Path
from unittest.mock import patch
from lookup_height_policy import same_heights,load_paths
ROOT=Path(__file__).resolve().parents[1]
class HeightPolicyTest(unittest.TestCase):
 def test_unchanged_and_changed_installed_heights(self):
  c=yaml.safe_load((ROOT/'src/deux_task_bringup/config/installation.yaml').read_text());b=json.loads((ROOT/'artifacts/dataset-barcode-pose/replay-height-baseline.json').read_text())['heights']
  self.assertTrue(same_heights(c,b)[0]);c['thor_geometry']['shoulder_height_from_floor_m']+=.02;self.assertFalse(same_heights(c,b)[0])
 def test_same_heights_do_not_invoke_full_validator(self):
  from types import SimpleNamespace as S
  c=yaml.safe_load((ROOT/'src/deux_task_bringup/config/installation.yaml').read_text());lib=S(entries={'example':{}})
  with patch('lookup_lift_scan.compile_library',side_effect=AssertionError('full validator called')),patch('lookup_lift_scan.compile_path',return_value='stored') as compile_one:
   paths,rejected=load_paths(lib,c);self.assertEqual(paths['example'],'stored');self.assertIsNotNone(compile_one.call_args.kwargs['reuse_clearance'])
 def test_changed_height_routes_to_validation(self):
  c=yaml.safe_load((ROOT/'src/deux_task_bringup/config/installation.yaml').read_text());c['thor_development']['conveyor_height_floor_m']+=.02
  with patch('lookup_lift_scan.compile_library',return_value=({'checked':1},{})) as check:
   self.assertEqual(load_paths(None,c),({'checked':1},{}));check.assert_called_once()
if __name__=='__main__':unittest.main()
