"""Production return coroutine with a synthetic controller (no ROS/hardware).

The production planner and measured URDF run unchanged. ROS message transport,
clock and controller are replaced; this is not a Jazzy communication test.
"""
import ast
import asyncio
from concurrent.futures import Future
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as S
import unittest
from uuid import uuid4
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from build_right_grasp_grid import Builder,ROOT
from deux_task_core.model import ARM_NAMES
from lookup_bt_support import prepare_plan,attention_plan


def done(value):
    f=Future();f.set_result(value);return f


class ReturnRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tree=ast.parse((ROOT/'tools/lookup_bt_server.py').read_text())
        source=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LookupBtServer')
        methods=[n for n in source.body if isinstance(n,ast.AsyncFunctionDef) and n.name in ('prepare','return_to_attention','settled_arms')]
        async def pause(node,delay):
            node.clock+=delay
            await asyncio.sleep(0)
        async def cancel(node,handles):node.canceled.extend(h for h in handles if h)
        scope=dict(np=np,time=S(monotonic=lambda:cls.node.clock),MoveToReady=S(Result=lambda **kw:S(**kw)),
            pause=pause,cancel_children=cancel,prepare_plan=prepare_plan,attention_plan=attention_plan,
            uuid4=uuid4,json=__import__('json'),ARM_NAMES=ARM_NAMES,CubicHermiteSpline=CubicHermiteSpline,
            JointTrajectory=lambda **kw:S(points=[],header=S(),**kw),JointTrajectoryPoint=lambda **kw:S(**kw),
            ros_time=lambda value,*args:value,Duration=object)
        extracted=ast.ClassDef(name='Server',bases=[],keywords=[],body=methods,decorator_list=[])
        exec(compile(ast.fix_missing_locations(ast.Module(body=[extracted],type_ignores=[])),'production-return','exec'),scope)
        cls.server=scope['Server'];cls.builder=Builder(ROOT/'src/deux_task_bringup/config/installation.yaml')

    def fixture(self,directory,mode='success',start_zero=False):
        node=self.server();node.attention_tolerance_rad=.05;node.ready_tolerance_rad=.05;node.left_ready_tolerance_rad=float(np.deg2rad(8.));type(self).node=node;node.clock=0.;node.config=self.builder.config
        node.library=S(ready=self.builder.ready.copy());node.actual=np.r_[node.library.ready[:7],node.library.ready[14:21]]
        if start_zero:node.actual[:]=0
        node.height_baseline=__import__('json').loads((ROOT/'artifacts/dataset-barcode-pose/replay-height-baseline.json').read_text())['heights']
        node.estop=False;node.out=Path(directory);node.canceled=[];node.dispatched=[]
        node.lift_scan_future=Future()  # Stow must not wait for unrelated grasp compilation.
        def guard(goal=None):
            if goal and goal.is_cancel_requested:raise InterruptedError('canceled')
            return node.actual.copy()
        node.prepare_guard=guard;node.measured=lambda:node.actual.copy();node.now=lambda:node.clock
        node.get_parameter=lambda key:S(value=.45 if key=='ready_velocity_cap' else .9)
        node.get_logger=lambda:S(error=lambda _:None)
        def submit(fn,*args):
            f=Future()
            try:f.set_result(fn(*args))
            except Exception as error:f.set_exception(error)
            return f
        node.pool=S(submit=submit);node.replay_goal=lambda tr:tr
        def send(tr):
            node.dispatched.append(tr)
            def result():
                node.actual=np.asarray(tr.points[-1].positions)
                if mode=='bad_final':node.actual[0]+=.08
                if mode=='relaxed_final':node.actual[0]+=.04
                return done(S(status=4,result=S(error_code=1 if mode=='controller_error' else 0)))
            return done(S(accepted=mode!='rejected',get_result_async=result))
        node.client=S(send_goal_async=send)
        return node

    def test_dispatches_reverse_and_confirms_attention(self):
        with tempfile.TemporaryDirectory() as directory:
            node=self.fixture(directory)
            result=asyncio.run(node.return_to_attention(S(is_cancel_requested=False)))
            self.assertTrue(result.success);self.assertEqual(len(node.dispatched),1)
            tr=node.dispatched[0];self.assertEqual(tr.joint_names,list(ARM_NAMES))
            np.testing.assert_allclose(tr.points[-1].positions,np.zeros(14))
            self.assertTrue(all(len(p.velocities)==14 and not any(p.velocities) for p in tr.points))
            self.assertFalse(node.canceled)

    def test_relaxed_attention_error_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            node=self.fixture(directory,'relaxed_final')
            result=asyncio.run(node.return_to_attention(S(is_cancel_requested=False)))
            self.assertTrue(result.success)
            self.assertAlmostEqual(node.actual[0],.04)

    def test_controller_and_feedback_failures_never_report_success(self):
        for mode in ('rejected','controller_error','bad_final'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as directory:
                node=self.fixture(directory,mode)
                with self.assertRaises(ValueError):asyncio.run(node.return_to_attention(S(is_cancel_requested=False)))
                self.assertTrue(node.canceled)

    def test_already_attention_and_operator_cancel_send_no_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            node=self.fixture(directory,start_zero=True)
            self.assertTrue(asyncio.run(node.return_to_attention(S(is_cancel_requested=False))).success)
            self.assertFalse(node.dispatched)
            with self.assertRaises(InterruptedError):asyncio.run(node.return_to_attention(S(is_cancel_requested=True)))
            self.assertFalse(node.dispatched)

    def test_coordinator_latches_shutdown_and_allows_only_return_retry(self):
        tree=ast.parse((ROOT/'tools/lookup_bt_node.py').read_text())
        source=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='LookupTaskNode')
        methods=[n for n in source.body if isinstance(n,ast.FunctionDef) and n.name in ('validate','validate_ready','validate_attention','set_repeat')]
        scope={}
        extracted=ast.ClassDef(name='Coordinator',bases=[],keywords=[],body=methods,decorator_list=[])
        exec(compile(ast.fix_missing_locations(ast.Module(body=[extracted],type_ignores=[])),'production-admission','exec'),scope)
        node=scope['Coordinator']();node.shutdown_requested=True;node.estop=False;node.repeat_enabled=False
        node.attention_client=S(server_is_ready=lambda:True);node.get_parameter=lambda key:S(value=True)
        node.robot_ready=lambda:None
        for method in (node.validate,node.validate_ready):
            with self.assertRaisesRegex(ValueError,'shutdown requested'):method(None)
        self.assertIsNone(node.validate_attention(None))
        response=node.set_repeat(S(data=True),S())
        self.assertFalse(response.success)


if __name__=='__main__':unittest.main()
