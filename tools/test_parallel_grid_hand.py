"""Deterministic virtual-clock tests: no ROS, hardware, sleeps or publishers."""
import asyncio
import unittest
from parallel_grid_hand import close_during_descent


class ParallelHandTest(unittest.TestCase):
    def run_case(self, initial=0., qualified=lambda elapsed: True, strict_timing=True):
        clock = [initial]; commands = []
        class Hand:
            def send(self, fraction): commands.append((clock[0], fraction))
        async def pause(dt): clock[0] += dt
        plan = dict(hand_schedule=dict(close_start_s=2.5, close_duration_s=.5, repeat_period_s=.02, close_fraction=1.),
                    events=dict(contact_s=3., close_complete_s=3., lift_start_s=3.))
        result = asyncio.run(close_during_descent(plan, 0., Hand(), now=lambda: clock[0], pause=pause,
                                                guard=lambda: None, qualified=qualified, strict_timing=strict_timing))
        return result, commands

    def test_commands_overlap_final_descent_and_finish_at_contact(self):
        result, commands = self.run_case()
        self.assertAlmostEqual(commands[0][0], 2.5)
        self.assertAlmostEqual(commands[-1][0], 3.)
        self.assertTrue(all(value == 1. for stamp, value in commands))
        self.assertFalse(result['physical_closure_verified'])

    def test_late_start_refused(self):
        with self.assertRaisesRegex(ValueError, 'missed hand closure start'):
            self.run_case(initial=2.55)

    def test_unqualified_feedback_refused_before_command(self):
        with self.assertRaisesRegex(ValueError, 'feedback not qualified'):
            self.run_case(qualified=lambda elapsed: False)

    def test_replay_can_log_timing_lateness_without_stopping(self):
        result, commands = self.run_case(initial=2.55, strict_timing=False)
        self.assertAlmostEqual(commands[0][0], 2.55)
        self.assertAlmostEqual(commands[-1][0], 3.)
        self.assertTrue(result['timing_warnings'])


if __name__ == '__main__': unittest.main()
