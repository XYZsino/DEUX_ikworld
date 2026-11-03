"""Shutdown ownership and fail-closed process handling; no ROS or hardware."""
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import stop_lookup_bt as stop


class StopLookupTests(unittest.TestCase):
    def test_launcher_identity(self):
        root=Path('/work/robot')
        self.assertTrue(stop.is_lookup_launcher(['/usr/bin/python3','/opt/ros/jazzy/bin/ros2','launch',str(root/'lookup_bt.thor.launch.py')],'/tmp',root))
        self.assertTrue(stop.is_lookup_launcher(['ros2','launch','lookup_bt.thor.launch.py'],str(root),root))
        for argv,cwd in [(['ros2','launch','control.launch.py'],str(root)),
                         (['bash','-c','ros2 launch lookup_bt.thor.launch.py'],str(root)),
                         (['ros2','launch','lookup_bt.thor.launch.py'],'/another/repo')]:
            self.assertFalse(stop.is_lookup_launcher(argv,cwd,root))

    def test_cancellation_failure_preserves_processes(self):
        with patch('sys.argv',['stop']),patch.object(stop,'find_launchers',return_value=[{'pid':12}]), \
             patch.object(stop,'cancel_actions',side_effect=TimeoutError('unconfirmed')),patch.object(stop.os,'kill') as kill:
            with self.assertRaises(TimeoutError):stop.main()
            kill.assert_not_called()

    def test_already_stopped_and_dry_run_do_not_touch_ros_or_processes(self):
        for argv,found in [(['stop'],[]),(['stop','--dry-run'],[{'pid':12}])]:
            with patch('sys.argv',argv),patch.object(stop,'find_launchers',return_value=found), \
                 patch.object(stop,'cancel_actions') as cancel,patch.object(stop.os,'kill') as kill:
                stop.main();cancel.assert_not_called();kill.assert_not_called()

    def test_success_only_signals_identified_launcher_after_cancel(self):
        with tempfile.TemporaryDirectory() as directory:
            events=[];record={'pid':12,'started':'1'}
            with patch('sys.argv',['stop']),patch.object(stop,'ROOT',Path(directory)), \
                 patch.object(stop,'find_launchers',return_value=[record]),patch.object(stop,'descendants',return_value=[]), \
                 patch.object(stop,'cancel_actions',side_effect=lambda _:events.append('cancel')), \
                 patch.object(stop,'return_to_attention',side_effect=lambda _:events.append('attention')), \
                 patch.object(stop,'same_process',side_effect=[True,False]), \
                 patch.object(stop.os,'kill',side_effect=lambda pid,sig:events.append((pid,sig))):
                stop.main()
            self.assertEqual(events,['cancel','attention',(12,stop.signal.SIGINT)])

    def test_pid_reuse_not_signaled(self):
        with patch.object(stop,'process',return_value={'pid':12,'started':'new'}):
            self.assertFalse(stop.same_process({'pid':12,'started':'old'}))

    def test_attention_failure_preserves_servers(self):
        with patch('sys.argv',['stop']),patch.object(stop,'find_launchers',return_value=[{'pid':12}]), \
             patch.object(stop,'cancel_actions'),patch.object(stop,'return_to_attention',side_effect=RuntimeError('motion failed')), \
             patch.object(stop.os,'kill') as kill:
            with self.assertRaisesRegex(RuntimeError,'motion failed'):stop.main()
            kill.assert_not_called()

    def test_explicit_no_stow_sends_no_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch('sys.argv',['stop','--no-stow']),patch.object(stop,'ROOT',Path(directory)), \
                 patch.object(stop,'find_launchers',return_value=[{'pid':12}]),patch.object(stop,'descendants',return_value=[]), \
                 patch.object(stop,'cancel_actions'),patch.object(stop,'return_to_attention') as motion, \
                 patch.object(stop,'same_process',return_value=False):
                stop.main();motion.assert_not_called()


if __name__=='__main__':unittest.main()
