"""Offline independent FK, timing, continuity and motion-limit tests."""
import unittest
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from build_fast_grasp_grid import FastBuilder, ROOT, target_position, V


class FastGridTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.b = FastBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
        cls.p = cls.b.plan_fast([.48, .03, .97])
        cls.s = CubicHermiteSpline(cls.p['times_s'], cls.p['positions_rad'], cls.p['velocities_rad_s'])

    def test_three_seconds_from_identical_ready(self):
        self.assertLessEqual(self.p['events']['contact_s'], 3.)
        np.testing.assert_allclose(self.s(0), self.b.ready[14:21], atol=1e-12)
        q = self.b.ready.copy(); q[14:21] = self.s(3.)
        np.testing.assert_allclose(self.b.robot.ee_fk('right', q)[:3, 3], [.48, .03, .97], atol=.0015)

    def test_follow_9cms_and_whole_rotation_cone(self):
        errors = []
        for t in np.linspace(1.5, 5., 701):
            q = self.b.ready.copy(); q[14:21] = self.s(t)
            pose = self.b.robot.ee_fk('right', q)
            errors.append(np.linalg.norm(pose[:3, 3]-target_position([.48, .03, .97], t, 1.5)))
            self.assertLessEqual(Rotation.from_matrix(pose[:3, :3]@self.b.rotation.T).magnitude(), np.deg2rad(30))
        self.assertLess(max(errors), .003)
        a = self.b.ready.copy(); a[14:21] = self.s(2.5)
        z = self.b.ready.copy(); z[14:21] = self.s(3.)
        displacement = self.b.robot.ee_fk('right', z)[:3, 3]-self.b.robot.ee_fk('right', a)[:3, 3]
        np.testing.assert_allclose(displacement[:2], [0., -.045], atol=.002)
        self.assertLess(displacement[2], -.005)
        for t in np.linspace(2.5, 3., 51):
            _, jac = self.b.fk_jac(self.s(t))
            self.assertLess(np.linalg.norm((jac[:3]@self.s(t, 1)-V)[:2]), .02)

    def test_closure_overlaps_descent_and_lift_has_no_bottom_dwell(self):
        schedule = self.p['hand_schedule']; events = self.p['events']
        self.assertEqual(schedule['close_start_s']+schedule['close_duration_s'], events['contact_s'])
        self.assertEqual(events['close_complete_s'], events['lift_start_s'])
        self.assertLess(events['close_start_s'], events['contact_s'])
        q0 = self.b.ready.copy(); q0[14:21] = self.s(3.)
        q1 = self.b.ready.copy(); q1[14:21] = self.s(3.25)
        p0 = self.b.robot.ee_fk('right', q0); p1 = self.b.robot.ee_fk('right', q1)
        self.assertGreater(p1[2, 3]-p0[2, 3], .002)
        np.testing.assert_allclose(p1[:2, 3]-p0[:2, 3], [0., -.0225], atol=.002)

    def test_velocity_acceleration_and_real_stop(self):
        t = np.linspace(0, 5., 5001)
        self.assertTrue(np.all(abs(self.s(t, 1)) <= .900001*self.b.robot.velocity[14:21]))
        self.assertTrue(np.all(abs(self.s(t, 2)) <= .900001*self.b.robot.acceleration[14:21]))
        self.assertTrue(np.all(self.s(t) >= self.b.lo-1e-10))
        self.assertTrue(np.all(self.s(t) <= self.b.hi+1e-10))
        np.testing.assert_allclose(self.s(0, 1), 0, atol=1e-10)
        np.testing.assert_allclose(self.s(5., 1), 0, atol=1e-10)

    def test_unreachable_rejected(self):
        with self.assertRaises(ValueError):
            self.b.relaxed(np.array([2., 2., .97]), self.b.ready)


if __name__ == '__main__': unittest.main()
