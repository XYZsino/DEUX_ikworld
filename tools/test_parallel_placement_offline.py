"""Pure real-URDF tests; no ROS, drivers, hardware, or command publishers."""
import sys,json,unittest
from pathlib import Path
from copy import deepcopy
import numpy as np,yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/deux_task_core'),str(ROOT/'src/deux_task_runtime'),str(ROOT/'tools')]
from deux_task_runtime.parallel_placement import build_parallel,validate_parallel
from deux_task_core.planning import Planner
from deux_task_core.model import RobotModel
from lookup_conveyor import validate_conveyor_path


class ParallelPlacementTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p=ROOT/'src/deux_task_bringup/config/installation.yaml';cls.config=yaml.safe_load(p.read_text())
        cls.config['robot']['urdf_path']=str((p.parent/cls.config['robot']['urdf_path']).resolve())
        cls.robot=RobotModel(cls.config['robot'])
    def start(self):
        q=np.array(self.config['home_joints']);q[14:21]=self.config['scan_right_joints']
        # Synthetic reader-aligned example, relative to installation home.
        q[[0,1,3,5]] += [.15,.1,-.1,.5]
        return q
    def test_arms_start_together_and_release_is_stationary(self):
        data=build_parallel(self.config,self.start());t=data['times'];q=data['path'];ph=data['phase_end_times']
        self.assertGreater(np.max(abs(q[1,:7]-q[0,:7])),1e-8)
        self.assertGreater(np.max(abs(q[1,14:21]-q[0,14:21])),1e-8)
        self.assertLess(data['report']['left_retreat_s'],ph[0]+1e-8)
        hold=q[(t>=ph[0]-1e-6)&(t<=ph[1]+1e-6)];arms=np.r_[np.arange(7),np.arange(14,21)]
        np.testing.assert_allclose(hold[:,arms],np.tile(hold[0,arms],(len(hold),1)))
        self.assertAlmostEqual(ph[1]-ph[0],self.config['hand_open_duration_s'])
        self.assertGreater(validate_conveyor_path(self.robot,q),0.)
    def test_preserves_existing_right_path_and_saves_left_only_interval(self):
        start=self.start();data=build_parallel(self.config,start)
        c=deepcopy(self.config);c['ik_backend']='scipy';virtual=start.copy();virtual[:7]=c['placement']['clear_left_joints']
        original=Planner(c).plan('PlaceApproach',virtual,np.eye(4))
        np.testing.assert_allclose(data['path'][:,14:21],original['path'][:,14:21])
        np.testing.assert_allclose(data['times'],original['times'])
        self.assertGreater(data['report']['sequential_left_duration_s'],0.)
    def test_slow_left_retreat_extends_right_approach_and_keeps_limits(self):
        start=self.start();start[0]=-2.4;c=deepcopy(self.config)
        c['robot']['acceleration_limits'][:7]=[.3]*7
        data=build_parallel(c,start)
        self.assertGreater(data['report']['right_approach_stretch'],1.)
        self.assertLessEqual(data['report']['velocity_limit_ratio'],1.001)
        self.assertLessEqual(data['report']['acceleration_limit_ratio'],1.01)
        self.assertLessEqual(np.max(np.diff(data['times'])),self.config['sample_period']*1.01)
    def test_rejects_missing_clearance_before_release_and_wrong_release_target(self):
        d=build_parallel(self.config,self.start())
        q=d['path'].copy();q[-1,0]+=.01
        with self.assertRaises(ValueError):validate_parallel(self.config,d['times'],q,d['phase_end_times'])
        q=d['path'].copy();i=np.flatnonzero(d['times']>=d['phase_end_times'][0]-1e-6)[0];q[i,14]+=.1
        with self.assertRaises(ValueError):validate_parallel(self.config,d['times'],q,d['phase_end_times'])
    def test_rejects_policy_change_and_honors_cancellation(self):
        c=deepcopy(self.config);c['placement']['continuous_motion']=False
        d=build_parallel(self.config,self.start())
        with self.assertRaises(ValueError):validate_parallel(c,d['times'],d['path'],d['phase_end_times'])
        with self.assertRaises(ValueError):build_parallel(c,self.start())
        with self.assertRaises(InterruptedError):build_parallel(self.config,self.start(),lambda:True)
    def executor_validate(self,enabled,plan,actual):
        # Execute the production admission method without importing ROS. The
        # path/model/policy checks are real; only ROS message plumbing is stubbed.
        import ast
        from types import SimpleNamespace as S
        from deux_task_core.planning import requires_object_pose
        module=ast.parse((ROOT/'src/deux_task_runtime/deux_task_runtime/execution_node.py').read_text())
        cls=next(x for x in module.body if isinstance(x,ast.ClassDef) and x.name=='ExecutionNode')
        method=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='validate')
        scope=dict(__package__='deux_task_runtime',np=np,RobotModel=RobotModel,JOINT_NAMES=[],
                   arrays=lambda tr,n:(tr.times,tr.path),seconds=lambda x:x,requires_object_pose=requires_object_pose)
        exec(compile(ast.Module(body=[method],type_ignores=[]),'production-executor-validate','exec'),scope)
        node=S(config=self.config,config_version='scene',completed=set(),check_model_version=lambda:None,
               grasp_age_guard=lambda p:None,measured=lambda:actual,now=lambda:100.1,
               get_parameter=lambda name:S(value=enabled if name=='parallel_reader_retreat' else False),
               position_error=lambda q,p,plan,phase:float(np.max(abs(q-p))))
        scope['validate'](node,S(plan=plan))
    def test_executor_opt_in_keeps_old_guard_and_rejects_bad_parallel_plan(self):
        from types import SimpleNamespace as S
        actual=self.start();d=build_parallel(self.config,actual)
        # Hand-only start discrepancies remain covered by separate real adapter
        # tests; use the exact command start for this arm admission test.
        plan=S(stage='PlaceApproach',ik_backend=self.config['ik_backend'],follows_conveyor=False,
               grasp_candidate_id='',phase_end_times=d['phase_end_times'],scene_version='scene',
               model_id=self.config['model_id'],plan_id='parallel',header=S(stamp=100.),valid_until=110.,
               synthetic=False,trajectory=S(times=d['times'],path=d['path']))
        self.executor_validate(True,plan,d['path'][0])
        with self.assertRaisesRegex(ValueError,'cleared left reader'):
            self.executor_validate(False,plan,d['path'][0])
        bad=deepcopy(plan);bad.trajectory.path[-1,0]+=.01
        with self.assertRaises(ValueError):self.executor_validate(True,bad,d['path'][0])
        moved=d['path'][0].copy();moved[0]+=.1
        with self.assertRaisesRegex(ValueError,'start mismatch'):
            self.executor_validate(True,plan,moved)

    def test_already_clear_remains_clear_and_config_is_unchanged(self):
        c=deepcopy(self.config);start=self.start();start[:7]=c['placement']['clear_left_joints']
        data=build_parallel(c,start);self.assertEqual(c,self.config)
        np.testing.assert_allclose(data['path'][:,:7],np.tile(start[:7],(len(data['path']),1)))


if __name__=='__main__':unittest.main()
