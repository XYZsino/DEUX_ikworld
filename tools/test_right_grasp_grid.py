"""Independent numerical checks for the offline grid generator (no hardware)."""
import unittest
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from build_right_grasp_grid import Builder, ROOT


class GridTest(unittest.TestCase):
    def setUp(self):
        self.b = Builder(ROOT/'src/deux_task_bringup/config/installation.yaml')

    def test_analytic_kinematics_matches_urdf_and_finite_difference(self):
        q = self.b.ready.copy()
        actual, jac = self.b.fk_jac(q[14:21])
        np.testing.assert_allclose(actual, self.b.robot.ee_fk('right', q), atol=1e-12)
        for j in range(7):
            perturbed = q[14:21].copy(); perturbed[j] += 1e-7
            t, _ = self.b.fk_jac(perturbed)
            np.testing.assert_allclose((t[:3, 3]-actual[:3, 3])/1e-7, jac[:3, j], atol=1e-7)

    def test_independent_full_grasp_path_and_limits(self):
        p = self.b.plan(.30, -.30)
        self.assertEqual([s['name'] for s in p['stages']], ['Approach', 'Descend', 'Close', 'Lift'])
        np.testing.assert_allclose(p['stages'][0]['positions_rad'][0], self.b.ready[14:21])
        previous = None
        for s in p['stages']:
            if previous is not None:
                np.testing.assert_allclose(previous, s['positions_rad'][0], atol=1e-12)
            previous = s['positions_rad'][-1]
            curve = CubicHermiteSpline(s['times_s'], s['positions_rad'], s['velocities_rad_s'])
            ts = np.linspace(0, s['duration_s'], 2001)
            for order, limit in [(1, self.b.robot.velocity[14:21]), (2, self.b.robot.acceleration[14:21])]:
                self.assertTrue(np.all(abs(curve(ts, order)) <= .500001*limit))
            for endpoint in [0., s['duration_s']]:
                np.testing.assert_allclose(curve(endpoint, 1), 0., atol=1e-10)
            if s['name'] in ('Descend', 'Lift'):
                for t in ts[::20]:
                    q = self.b.ready.copy(); q[14:21] = curve(t)
                    pose = self.b.robot.ee_fk('right', q)
                    np.testing.assert_allclose(pose[:2, 3], [.3, -.3], atol=.002)
        for index, height in [(1, .97), (3, 1.07)]:
            q = self.b.ready.copy(); q[14:21] = p['stages'][index]['positions_rad'][-1]
            self.assertAlmostEqual(self.b.robot.ee_fk('right', q)[2, 3], height, delta=.001)

    def test_far_target_rejected(self):
        with self.assertRaises(ValueError):
            self.b.solve(np.array([2., 2., .97]), self.b.ready)


if __name__ == '__main__':
    unittest.main()
