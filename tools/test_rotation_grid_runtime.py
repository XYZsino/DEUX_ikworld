"""Actual saved-library regression for XY-first, soft yaw rotation ranking."""
import unittest
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from grid_grasp_runtime import GridLibrary, ROOT


class RotationSelectionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=ROOT/'src/deux_task_bringup/config/installation.yaml'
        config=yaml.safe_load(path.read_text())
        config['robot']['urdf_path']=str((path.parent/config['robot']['urdf_path']).resolve())
        cls.library=GridLibrary(config,recommended=False,
            directory=ROOT/'artifacts/right_grasp_grid_rotations',yaw_secondary=True,load_rotation_paths=False)

    def pose(self,yaw,roll=0,pitch=0):
        p=np.eye(4);p[:3,3]=[.42,.48,.927]
        p[:3,:3]=Rotation.from_euler('xyz',[roll,pitch,yaw],degrees=True).as_matrix()
        return p

    def select(self,p):
        return self.library.select(p,np.zeros(4),100.,100.)

    def test_coverage_and_original_schedule(self):
        self.assertEqual(len(self.library.entries),1587)
        self.assertEqual(len(self.library.cell_options),118)
        a=self.select(self.pose(20))
        self.assertAlmostEqual(a['contact_at_s']-a['start_at_s'],3.)
        self.assertEqual(a['contact_xyz_m'][2],.97)

    def test_yaw_changes_rotation_without_changing_xy_or_encounter_time(self):
        a=self.select(self.pose(-20));b=self.select(self.pose(20))
        self.assertEqual(a['contact_xyz_m'],b['contact_xyz_m'])
        self.assertEqual(a['contact_at_s'],b['contact_at_s'])
        self.assertNotEqual(a['cell_id'],b['cell_id'])
        self.assertGreater(a['rotation_candidates_at_cell'],1)

    def test_observed_roll_pitch_do_not_rank_candidates(self):
        a=self.select(self.pose(20));b=self.select(self.pose(20,roll=25,pitch=-20))
        self.assertEqual(a['cell_id'],b['cell_id'])

    def test_large_and_wrapped_yaw_never_disqualify_spatial_cell(self):
        base=self.select(self.pose(0))
        for angle in (150,179,-179,-150):
            chosen=self.select(self.pose(angle))
            self.assertEqual(chosen['contact_xyz_m'],base['contact_xyz_m'])
            self.assertTrue(chosen['rotation_is_soft_preference'])

    def test_matches_independent_so3_rank_at_chosen_cell(self):
        pose=self.pose(23);chosen=self.select(pose)
        target=Rotation.from_euler('z',23,degrees=True).as_matrix()@self.library.offset[:3,:3]
        keys=self.library.cell_options[tuple(chosen['contact_xyz_m'])]
        errors={key:Rotation.from_matrix(self.library.contact_rotations[key]@target.T).magnitude() for key in keys}
        self.assertAlmostEqual(errors[chosen['cell_id']],min(errors.values()),places=9)


if __name__=='__main__':unittest.main()
