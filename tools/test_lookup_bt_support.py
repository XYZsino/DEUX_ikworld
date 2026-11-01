"""Offline handoff and actual-model preparation/ScanPose integration checks."""
import json,unittest
from copy import deepcopy
from types import SimpleNamespace as S
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_bt_support import ROOT,prepare_plan,validate_handoff
from deux_task_core.planning import Planner


class LookupSupportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=ROOT/'src/deux_task_bringup/config/installation.yaml'
        cls.config=yaml.safe_load(path.read_text())
        cls.config['robot']['urdf_path']=str((path.parent/cls.config['robot']['urdf_path']).resolve())
        cls.lib=ClassifiedGridLibrary(cls.config,directory=ROOT/'artifacts/right_grasp_grid_classified',
            stationary_left_file=ROOT/'artifacts/thor-transverse-deployment/stationary-left.json',
            yaw_secondary=True,load_rotation_paths=False)

    def test_faster_ready_preserves_geometry_and_respects_cubic_limits(self):
        ready=self.lib.ready.copy();ready[:7]=self.config['home_joints'][:7]
        actual=ready.copy()
        actual[:7]=json.loads((ROOT/'artifacts/dataset-barcode-pose/reading-pose.json').read_text())['left_joints_rad']
        actual[14:21]=self.config['scan_right_joints']
        arms=np.r_[actual[:7],actual[14:21]]
        slow=prepare_plan(self.config,ready,arms)
        fast=prepare_plan(self.config,ready,arms,velocity_cap=.45,acceleration_cap=.9)
        np.testing.assert_allclose(fast['path'],slow['path'])
        self.assertLess(fast['times'][-1],slow['times'][-1]*.65)
        from deux_task_core.model import RobotModel
        robot=RobotModel(self.config['robot']);indices=np.r_[np.arange(7),np.arange(14,21)]
        delta=abs(np.diff(fast['path'][:,indices],axis=0));dt=np.diff(fast['times'])[:,None]
        self.assertTrue(np.all(1.5*delta/dt<=np.minimum(robot.velocity[indices],.45)+1e-9))
        self.assertTrue(np.all(6*delta/dt**2<=np.minimum(robot.acceleration[indices],.9)+1e-9))
        print('Scan return duration:',slow['times'][-1],'->',fast['times'][-1])

    def result(self):
        return S(success=True,path_completed=True,request_id='request',target_id='bag',cell_id='cell',
                 grasp_mode='normal',contact_grasp=S(header=S(frame_id='base_link')),physical_grasp_verified=False)

    def test_handoff_contract(self):
        self.assertEqual(validate_handoff(self.result(),'request','bag'),'bag')
        for key,value in [('success',False),('path_completed',False),('request_id','old'),('target_id',''),
                          ('cell_id',''),('grasp_mode','invalid'),('physical_grasp_verified',True)]:
            result=self.result();setattr(result,key,value)
            with self.subTest(key=key),self.assertRaises(ValueError):validate_handoff(result,'request')
        with self.assertRaises(ValueError):validate_handoff(self.result(),'request','different-bag')
        result=self.result();result.contact_grasp.header.frame_id='camera'
        with self.assertRaises(ValueError):validate_handoff(result,'request')

    def test_already_ready_is_checked_and_invalid_joints_rejected(self):
        q=self.lib.ready;arms=np.r_[q[:7],q[14:21]]
        self.assertTrue(prepare_plan(self.config,q,arms)['no_motion'])
        with self.assertRaises(ValueError):prepare_plan(self.config,q,np.full(14,np.nan))

    def test_installation_left_home_conveyor_only_transfer(self):
        ready=self.lib.ready.copy();ready[:7]=self.config['home_joints'][:7]
        actual=np.r_[self.lib.ready[:7],self.lib.ready[14:21]]
        plan=prepare_plan(self.config,ready,actual)
        np.testing.assert_allclose(plan['path'][-1,:7],self.config['home_joints'][:7])
        np.testing.assert_allclose(plan['path'][:,14:21],np.tile(ready[14:21],(len(plan['path']),1)))
        self.assertGreater(plan['minimum_clearance'],0.)

    def test_restore_reader_left_arm_after_placement(self):
        q=self.lib.ready.copy()
        q[:7]=self.config['placement']['clear_left_joints']
        q[14:21]=self.config['placement']['fixed_joint_waypoints']['Retreat'][-1]
        back=prepare_plan(self.config,self.lib.ready,np.r_[q[:7],q[14:21]])
        np.testing.assert_allclose(back['path'][-1,:7],self.lib.ready[:7])
        np.testing.assert_allclose(back['path'][-1,14:21],self.lib.ready[14:21])

    def test_normal_and_transverse_stop_to_scan_and_return_ready(self):
        for yaw in (0.,90.,-90.):
            pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix();pose[:3,3]=[.6,.5,.97]
            selected=self.lib.select(pose,np.zeros(4),100.,100.)
            q=self.lib.ready.copy();q[14:21]=self.lib.curves[selected['cell_id']](5.)
            with self.subTest(yaw=yaw):
                config=deepcopy(self.config);config['ik_backend']='scipy'  # ScanPose is a fixed joint path; no IK.
                plan=Planner(config).plan('ScanPose',q,np.eye(4))
                self.assertGreater(plan['times'][-1],0.)
                final=plan['path'][-1]
                actual=np.r_[final[:7],final[14:21]]
                back=prepare_plan(self.config,self.lib.ready,actual)
                self.assertFalse(back['no_motion'])
                np.testing.assert_allclose(back['path'][-1,:7],self.lib.ready[:7])
                np.testing.assert_allclose(back['path'][-1,14:21],self.lib.ready[14:21])


if __name__=='__main__':unittest.main()
