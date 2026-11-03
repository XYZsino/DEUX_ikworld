"""Numerical checks for the fixed-rotation solver and cone interpretation."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from explore_grasp_rotations import FixedRotationBuilder, ROOT, rotations
from build_fast_grasp_grid import FastBuilder


class RotationSearchTest(unittest.TestCase):
    def test_early_extrema_screen_preserves_complete_accepted_path(self):
        b = FixedRotationBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
        r = next(r for r in rotations() if r['id'] == 'axis04_15')
        b.target_rotation = Rotation.from_rotvec(np.deg2rad(r['rotvec_deg'])).as_matrix()@b.rotation
        b.rng = np.random.default_rng(970)
        screened = b.plan_fast([.45, -.06, .97], 1.8)
        b.rng = np.random.default_rng(970)
        original = FastBuilder.plan_fast(b, [.45, -.06, .97], 1.8)
        np.testing.assert_array_equal(screened['positions_rad'], original['positions_rad'])
        np.testing.assert_array_equal(screened['velocities_rad_s'], original['velocities_rad_s'])
        self.assertEqual(screened['checks'], original['checks'])

    def test_axis_angle_samples_stay_in_total_cone(self):
        samples = rotations()
        self.assertEqual(len(samples), 53)
        matrices = [Rotation.from_rotvec(np.deg2rad(r['rotvec_deg'])).as_matrix() for r in samples]
        self.assertEqual(len({np.round(m, 10).tobytes() for m in matrices}), 53)
        for m in matrices:
            self.assertLessEqual(np.degrees(Rotation.from_matrix(m).magnitude()), 29.000001)
        # Three simultaneous 30deg Euler rotations would violate the user's total cone.
        self.assertGreater(np.degrees(Rotation.from_euler('xyz', [30., 30., 30.], degrees=True).magnitude()), 30.)

    def test_solver_tracks_requested_rotation_and_preserves_reference(self):
        b = FixedRotationBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
        original = b.rotation.copy()
        b.target_rotation = Rotation.from_rotvec(np.deg2rad([0., -29., 0.])).as_matrix()@original
        q = b.relaxed(np.array([.48, .03, .97]), b.ready, attempts=2)
        pose = b.robot.ee_fk('right', q)
        self.assertLess(np.linalg.norm(pose[:3, 3]-[.48, .03, .97]), .001)
        self.assertLess(np.degrees(Rotation.from_matrix(pose[:3, :3]@b.target_rotation.T).magnitude()), 1.)
        np.testing.assert_array_equal(b.rotation, original)
        with self.assertRaises(ValueError):
            b.relaxed(np.array([2., 2., .97]), b.ready)
        np.testing.assert_array_equal(b.rotation, original)
        np.testing.assert_array_equal(q[:14], b.ready[:14])
        np.testing.assert_array_equal(q[21:], b.ready[21:])


if __name__ == '__main__':
    unittest.main()
