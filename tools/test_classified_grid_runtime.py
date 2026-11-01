"""Actual-library checks for transverse class, symmetry, fixed-axis admission."""
import unittest
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from classified_grid_runtime import ClassifiedGridLibrary,transverse_yaw,axis_error_degrees
from grid_grasp_runtime import GridLibrary,ROOT


class ClassifiedTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=ROOT/'src/deux_task_bringup/config/installation.yaml';config=yaml.safe_load(path.read_text())
        config['robot']['urdf_path']=str((path.parent/config['robot']['urdf_path']).resolve())
        cls.lib=ClassifiedGridLibrary(config,recommended=False,directory=ROOT/'artifacts/right_grasp_grid_classified',
                                     yaw_secondary=True,load_rotation_paths=False)

    def pose(self,yaw,x=.6):
        p=np.eye(4);p[:3,:3]=Rotation.from_euler('z',yaw,degrees=True).as_matrix();p[:3,3]=[x,.5,.927];return p

    def select(self,yaw,x=.6):return self.lib.select(self.pose(yaw,x),np.zeros(4),100.,100.)

    def test_class_boundaries_and_wrapping(self):
        for a in [60,90,120,-60,-90,-120,270,-270]:self.assertTrue(transverse_yaw(a))
        for a in [0,59.999,-59.999,120.001,-120.001,180]:self.assertFalse(transverse_yaw(a))

    def test_plus_minus_ninety_use_identical_axis_preserving_path(self):
        plus=self.select(90);minus=self.select(-90)
        self.assertEqual(plus['cell_id'],minus['cell_id']);self.assertEqual(plus['contact_at_s'],minus['contact_at_s'])
        self.assertTrue(plus['front_back_reversed']);self.assertFalse(minus['front_back_reversed'])
        self.assertEqual(plus['grasp_mode'],'transverse_axis_locked')
        self.assertLess(plus['axis_error_deg'],.5);self.assertGreater(plus['rotation_error_deg'],80.)

    def test_all_transverse_angles_have_a_model_path_at_x_060(self):
        for yaw in [-120,-117.5,-105,-97,-90,-75,-60,60,72.5,90,105,120]:
            selected=self.select(yaw);self.assertLessEqual(selected['axis_error_deg'],3.)
            self.assertEqual(selected['contact_at_s']-selected['start_at_s'],3.)
            self.assertTrue(selected['orientation_exception_applied'])

    def test_normal_selection_is_unchanged(self):
        p=self.pose(-5);expected=GridLibrary.select_with_yaw(self.lib.normal_view,
            self.lib.grasp_now(p,np.zeros(4),100.,100.)[0],p,100.,100.,.8)
        actual=self.select(-5)
        self.assertEqual(actual['cell_id'],expected['cell_id']);self.assertEqual(actual['start_at_s'],expected['start_at_s'])
        self.assertFalse(actual['orientation_exception_applied'])

    def test_special_class_does_not_fall_back_to_normal(self):
        normal=self.select(0,x=.36);self.assertEqual(normal['grasp_mode'],'normal')
        with self.assertRaisesRegex(ValueError,'no normal fallback'):self.select(90,x=.36)

    def test_pre_dispatch_axis_check_allows_front_back_flip_but_not_direction_change(self):
        selected=self.select(-90);self.lib.live_error(selected,self.pose(90),np.zeros(4),100.,100.)
        with self.assertRaisesRegex(ValueError,'axis changed'):self.lib.live_error(selected,self.pose(-80),np.zeros(4),100.,100.)
        with self.assertRaisesRegex(ValueError,'class changed'):self.lib.live_error(selected,self.pose(0),np.zeros(4),100.,100.)

    def test_fixed_axis_remains_aligned_through_descent_and_lift(self):
        selected=self.select(-97);p=self.lib.entries[selected['cell_id']];curve=self.lib.curves[selected['cell_id']]
        for t in np.linspace(p['events']['approach_complete_s'],5.,201):
            q=self.lib.ready.copy();q[14:21]=curve(t)
            relative=self.lib.robot.ee_fk('right',q)[:3,:3]@self.lib.offset[:3,:3].T
            yaw=np.degrees(np.arctan2(relative[1,0],relative[0,0]))
            self.assertLessEqual(axis_error_degrees(yaw,-97),3.)


if __name__=='__main__':unittest.main()
