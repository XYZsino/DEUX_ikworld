"""Model-based checks of the production replay extension, without ROS."""
import unittest,json
from pathlib import Path
import numpy as np,yaml
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_lift_scan import compile_path,VIA,restrict_selection
ROOT=Path(__file__).resolve().parents[1]
class LiftScanTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  p=ROOT/'src/deux_task_bringup/config/installation.yaml';cls.config=yaml.safe_load(p.read_text());cls.config['robot']['urdf_path']=str((p.parent/cls.config['robot']['urdf_path']).resolve())
  cls.lib=ClassifiedGridLibrary(cls.config,directory=ROOT/'artifacts/right_grasp_grid_classified',recommended=False,yaw_secondary=True,load_rotation_paths=False)
  cls.lib.ready[:7]=cls.config['home_joints'][:7]
 def test_accepted_reference_c2_prefix_and_endpoint(self):
  key='x+021_y-001_axis09_29';plan,curve,info=compile_path(self.lib,key,self.config)
  np.testing.assert_allclose(curve(np.linspace(0,4,101)),self.lib.curves[key](np.linspace(0,4,101)),atol=1e-12)
  for derivative in (0,1,2):
   expected=self.lib.curves[key](np.nextafter(4.,-np.inf) if derivative==2 else 4.,derivative)
   np.testing.assert_allclose(curve.suffix(0.,derivative),expected,atol=1e-10)
  np.testing.assert_allclose(curve(info['waypoint_at_s']),VIA,atol=1e-10)
  np.testing.assert_allclose(curve(curve.x[-1]),self.config['scan_right_joints'],atol=1e-10)
  self.assertEqual(info['duration_s'],8.);self.assertEqual(plan['hand_schedule'],self.lib.entries[key]['hand_schedule']);self.assertGreater(info['minimum_conveyor_clearance_m'],0.)
 def test_transverse_and_far_edge_connections(self):
  for key in [next(k for k,p in self.lib.entries.items() if p.get('selection_class')=='transverse'),next(iter(self.lib.entries))]:
   plan,curve,info=compile_path(self.lib,key,self.config)
   self.assertEqual(plan['events']['contact_s'],3.);self.assertTrue(info['scan_completed'])
 def test_parallel_left_scan_uses_one_14_joint_clock_and_preserves_right(self):
  from deux_task_core.model import ARM_NAMES
  target=json.loads((ROOT/'artifacts/dataset-barcode-pose/reading-pose.json').read_text())['left_joints_rad']
  keys=['x+021_y-001_axis09_29',next(k for k,p in self.lib.entries.items() if p.get('selection_class')=='transverse')]
  for key in keys:
   original,right,old_info=compile_path(self.lib,key,self.config)
   plan,curve,info=compile_path(self.lib,key,self.config,left_target=target)
   t=np.asarray(plan['times_s']);q=np.asarray(plan['positions_rad']);v=np.asarray(plan['velocities_rad_s'])
   self.assertEqual(plan['joint_names'],list(ARM_NAMES));self.assertEqual(q.shape,(len(t),14))
   np.testing.assert_allclose(q[:,7:],right(t),atol=1e-12)
   np.testing.assert_allclose(q[t<=4.,:7],np.tile(self.lib.ready[:7],(sum(t<=4.),1)),atol=1e-12)
   np.testing.assert_allclose(q[t>=info['waypoint_at_s'],:7],np.tile(target,(sum(t>=info['waypoint_at_s']),1)),atol=1e-12)
   self.assertGreater(np.linalg.norm(curve.left_curve(4.5,1)),.01)
   for time in [4.,info['waypoint_at_s']]:
    np.testing.assert_allclose(curve.left_curve(time,1),np.zeros(7),atol=1e-10)
    np.testing.assert_allclose(curve.left_curve(time,2),np.zeros(7),atol=1e-10)
   self.assertEqual(plan['hand_schedule'],original['hand_schedule']);self.assertGreater(info['minimum_conveyor_clearance_m'],0.)
 def test_production_server_accepts_moving_left_only_after_commit(self):
  import ast
  from types import SimpleNamespace as S
  source=ast.parse((ROOT/'tools/lookup_bt_server.py').read_text())
  klass=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='LookupBtServer')
  methods=[n for n in klass.body if isinstance(n,ast.FunctionDef) and n.name in ('gate','validate_replay')]
  class Parent:
   def validate_replay(self,*args):return self.actual
  scope=dict(np=np,Parent=Parent)
  extracted=ast.ClassDef(name='Server',bases=[ast.Name(id='Parent',ctx=ast.Load())],keywords=[],body=methods,decorator_list=[])
  tree=ast.fix_missing_locations(ast.Module(body=[extracted],type_ignores=[]));exec(compile(tree,'production-lookup-server','exec'),scope)
  target=json.loads((ROOT/'artifacts/dataset-barcode-pose/reading-pose.json').read_text())['left_joints_rad']
  _,curve,info=compile_path(self.lib,'x+021_y-001_axis09_29',self.config,left_target=target)
  node=scope['Server']();node.config=self.config;node.library=self.lib;node.locked_selection={};node.now=lambda:5.
  node.actual=np.r_[curve.left_curve(5.),curve(5.)];node.prepare_guard=lambda goal:node.actual;warnings=[];node.warn_execution=warnings.append
  np.testing.assert_allclose(node.gate(None),node.actual);node.validate_replay(None,curve,0.);self.assertFalse(warnings)
  node.actual[0]+=.5;node.validate_replay(None,curve,0.);self.assertIn('parallel left scan tracking error',warnings)
  node.locked_selection=None
  with self.assertRaises(ValueError):node.gate(None)
 def test_rejected_paths_removed_only_from_selection_views(self):
  import copy
  lib=copy.copy(self.lib);lib.normal_view=copy.copy(self.lib.normal_view)
  old=dict(lib.entries);normal=next(iter(lib.normal_options.values()))[0];transverse=next(iter(lib.transverse_options.values()))[0]
  restrict_selection(lib,{normal:None,transverse:None})
  self.assertEqual({k for values in lib.normal_view.cell_options.values() for k in values},{normal})
  self.assertEqual({k for values in lib.transverse_options.values() for k in values},{transverse});self.assertEqual(set(lib.entries),set(old))
if __name__=='__main__':unittest.main()
