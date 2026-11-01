"""Offline regression: real model screening, fixed timing and no ROS commands."""
import asyncio
from pathlib import Path
import unittest
import numpy as np
import yaml
from scipy.interpolate import CubicHermiteSpline
from roi_grasp_planner import RoiPlanner, load_policy, encounter
from parallel_grid_hand import close_during_descent


class RoiGraspTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.policy = load_policy()
        path = Path(__file__).resolve().parents[1]/'src/deux_task_bringup/config/installation.yaml'
        cls.config = yaml.safe_load(path.read_text())
        cls.config['robot']['urdf_path'] = str((path.parent/cls.config['robot']['urdf_path']).resolve())
        cls.builder = RoiPlanner(cls.config, cls.policy)
        cls.alignment = cls.builder.align(cls.builder.ready, .48)
        cls.aligned = cls.builder.ready.copy(); cls.aligned[14:21] = cls.alignment['positions_rad'][-1]
        cls.plan = cls.builder.grasp(cls.aligned, [.48, .03, .97], 1.)

    def test_absolute_contact_and_parallel_close(self):
        cells = dict(a=dict(contact_xyz_m=[.48, .03, .97]))
        s = encounter([.48, .30, 99.], .481, cells, 1., 100., self.policy)
        self.assertAlmostEqual(s['contact_at_s'], 103.)
        self.assertAlmostEqual(s['start_at_s'], 102.)
        p = self.plan
        self.assertEqual(p['events']['close_start_s'], .5)
        self.assertEqual(p['events']['close_complete_s'], p['events']['contact_s'])
        self.assertEqual(p['events']['lift_start_s'], p['events']['contact_s'])

    def test_planning_budget_selects_a_cell_that_survives_computation(self):
        cells = dict(early=dict(contact_xyz_m=[.48, .18, .97]), later=dict(contact_xyz_m=[.48, .12, .97]))
        planning = dict(self.policy, dispatch_lead_s=self.policy['dispatch_lead_s']+.5)
        selected = encounter([.48, .30], .48, cells, 1., 100., planning)
        self.assertEqual(selected['cell_id'], 'later')
        refreshed = encounter([.48, .30-.09*.5], .48, {'later': cells['later']}, 1., 100.5, self.policy)
        self.assertAlmostEqual(selected['contact_at_s'], refreshed['contact_at_s'])

    def test_entry_alignment_and_lead_are_separate_gates(self):
        cells = dict(a=dict(contact_xyz_m=[.48, .03, .97]))
        for xy, x, pattern in [([.48, .5], .48, 'outside'), ([.48, .3], .40, 'alignment'),
                                ([.48, .06], .48, 'lead')]:
            with self.assertRaisesRegex(ValueError, pattern): encounter(xy, x, cells, 1., 100., self.policy)

    def test_actual_model_path_has_zero_endpoints_and_contact_velocity(self):
        p = self.plan
        c = CubicHermiteSpline(p['times_s'], p['positions_rad'], p['velocities_rad_s'])
        np.testing.assert_allclose(c(0), self.aligned[14:21], atol=1e-12)
        np.testing.assert_allclose(c(0, 1), 0., atol=1e-12)
        np.testing.assert_allclose(c(p['times_s'][-1], 1), 0., atol=1e-10)
        q = self.aligned.copy(); q[14:21] = c(1.)
        np.testing.assert_allclose(self.builder.robot.ee_fk('right', q)[:3, 3], [.48, .03, .97], atol=.001)
        _, jac = self.builder.fk_jac(q[14:21])
        np.testing.assert_allclose(jac[:3]@c(1., 1), [0, -.09, 0], atol=.003)
        self.assertLess(p['checks']['limit_ratios']['2'], .9)
        self.assertLess(p['checks']['rotation_error_rad'], np.deg2rad(2))

    def test_subsequent_alignment_holds_y_z(self):
        p = RoiPlanner(self.config, self.policy).align(self.aligned, .50)
        c = CubicHermiteSpline(p['times_s'], p['positions_rad'], p['velocities_rad_s'])
        initial = self.builder.robot.ee_fk('right', self.aligned)[:3, 3]
        for t in np.linspace(0., p['times_s'][-1], 25):
            q = self.aligned.copy(); q[14:21] = c(t)
            np.testing.assert_allclose(self.builder.robot.ee_fk('right', q)[1:3, 3], initial[1:3], atol=.003)

    def test_hand_feedback_fault_logs_and_keeps_schedule(self):
        clock = [100.]; commands = []
        class Hand:
            def send(self, value): commands.append(clock[0])
        async def pause(dt): clock[0] += dt
        r = asyncio.run(close_during_descent(self.plan, 100., Hand(), now=lambda: clock[0], pause=pause,
            guard=lambda: None, qualified=lambda t: False, strict_timing=False))
        self.assertTrue(r['timing_warnings'])
        self.assertAlmostEqual(commands[0], 100.5)
        self.assertAlmostEqual(commands[-1], 101.)

    def test_hand_driver_refusal_is_logged_not_faked_as_a_command(self):
        clock = [0.]
        class Hand:
            def send(self, value): raise ValueError('driver unavailable')
        async def pause(dt): clock[0] += dt
        r = asyncio.run(close_during_descent(self.plan, 0., Hand(), now=lambda: clock[0], pause=pause,
            guard=lambda: None, qualified=lambda t: True, strict_timing=False))
        self.assertEqual(r['commands'], 0)
        self.assertGreater(r['attempts'], 0)
        self.assertIsNone(r['first_command_s'])

    def test_explicit_stop_is_not_swallowed(self):
        clock = [0.]
        async def pause(dt): clock[0] += dt
        def stop(): raise InterruptedError('operator canceled')
        with self.assertRaises(InterruptedError):
            asyncio.run(close_during_descent(self.plan, 0., None, now=lambda: clock[0], pause=pause,
                guard=stop, qualified=lambda t: True, strict_timing=False))


if __name__ == '__main__': unittest.main()
