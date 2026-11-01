"""Production fixed-pose builder, real recorded joints and real URDF; no ROS."""
import unittest,json
from pathlib import Path
from copy import deepcopy
import numpy as np,yaml
from deux_task_runtime.dataset_reading_pose import load_pose,build_reading_plan,validate_endpoint
from deux_task_runtime.parallel_placement import build_parallel
from deux_task_core.model import RobotModel
from lookup_conveyor import validate_conveyor_path
ROOT=Path(__file__).resolve().parents[1]
class DatasetPoseTest(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  p=ROOT/'src/deux_task_bringup/config/installation.yaml';cls.c=yaml.safe_load(p.read_text());cls.c['robot']['urdf_path']=str((p.parent/cls.c['robot']['urdf_path']).resolve());cls.pose=load_pose(ROOT/'artifacts/dataset-barcode-pose/reading-pose.json');cls.robot=RobotModel(cls.c['robot'])
 def test_recorded_left_endpoint_and_unchanged_right_entire_path(self):
  q=np.array(self.c['home_joints']);q[14:21]=self.c['scan_right_joints'];plan=build_reading_plan(self.c,self.pose,q);validate_endpoint(self.pose,plan['path'])
  np.testing.assert_allclose(plan['path'][:,14:21],np.tile(q[14:21],(len(plan['path']),1)),atol=1e-12)
  self.assertAlmostEqual(plan['path'][-1,5],self.pose['left_joints_rad'][5]);self.assertGreater(validate_conveyor_path(self.robot,plan['path']),0.)
  t=plan['times'];v=np.diff(plan['path'],axis=0)/np.diff(t)[:,None];a=np.diff(np.vstack([np.zeros(28),v,np.zeros(28)]),axis=0)/np.r_[np.diff(t),np.diff(t)[-1]][:,None]
  self.assertLessEqual(np.max(abs(v)/self.robot.velocity),1.001);self.assertLessEqual(np.max(abs(a)/self.robot.acceleration),1.01)
 def test_parallel_withdrawal_starts_from_recorded_reading_pose(self):
  q=np.array(self.c['home_joints']);q[:7]=self.pose['left_joints_rad'];q[14:21]=self.c['scan_right_joints'];p=build_parallel(self.c,q);self.assertGreater(validate_conveyor_path(self.robot,p['path']),0.)
  np.testing.assert_allclose(p['path'][0,:7],q[:7]);np.testing.assert_allclose(p['path'][-1,:7],self.c['placement']['clear_left_joints'])
 def test_fixed_pose_skips_visual_barcode_gates_but_dynamic_mode_keeps_them(self):
  import ast
  from types import SimpleNamespace as S
  for path,method in [('src/deux_task_runtime/deux_task_runtime/motion_planner_node.py','validate_label'),('src/deux_task_runtime/deux_task_runtime/execution_node.py','barcode_guard')]:
   source=ast.parse((ROOT/path).read_text());fn=next(n for n in ast.walk(source) if isinstance(n,ast.FunctionDef) and n.name==method);scope={}
   exec(compile(ast.Module(body=[fn],type_ignores=[]),'production-barcode-gate','exec'),scope)
   node=S(dataset_reading_pose=self.pose,config={},latest_barcode_target=None);request=S(barcode_target=S(request_id='',target_id=''))
   self.assertIsNone(scope[method](node,request));node.dataset_reading_pose=None
   with self.assertRaises(ValueError):scope[method](node,request)
 def test_fixed_pose_coordinator_never_waits_for_camera_barcode(self):
  import ast,asyncio
  from types import SimpleNamespace as S
  source=ast.parse((ROOT/'tools/lookup_bt_node.py').read_text());fn=next(n for n in ast.walk(source) if isinstance(n,ast.AsyncFunctionDef) and n.name=='align');events=[]
  async def stage(name,*args):events.append(name)
  async def wait(*args):raise AssertionError('camera barcode acquisition unexpectedly required')
  node=S(dataset_reading_pose=self.pose,scan_attempt=0,set_state=events.append,stage=stage,await_condition=wait)
  scope=dict(self=node,goal=None,version='scene');exec(compile(ast.Module(body=[fn],type_ignores=[]),'production-fixed-alignment','exec'),scope);asyncio.run(scope['align']());self.assertEqual(events,['AlignBarcode'])
 def test_rejects_target_mismatch_and_joint_limit(self):
  q=np.array(self.c['home_joints']);p=build_reading_plan(self.c,self.pose,q);p['path'][-1,14]+=.1
  with self.assertRaises(ValueError):validate_endpoint(self.pose,p['path'])
  bad=deepcopy(self.pose);bad['left_joints_rad'][0]=99.
  with self.assertRaises(ValueError):build_reading_plan(self.c,bad,q)
if __name__=='__main__':unittest.main()
