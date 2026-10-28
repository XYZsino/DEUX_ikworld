"""Real DDS/Action and HTTP tests; no hardware or physical task success claims."""
import json
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import pytest
rclpy = pytest.importorskip('rclpy')
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.task import Future
from deux_task_msgs.action import RunTask, MoveToReady
from deux_task_msgs.msg import TaskStatus
from deux_task_gui.node import TaskGui


class Harness:
    next_domain = 174

    def __init__(self):
        rclpy.init(domain_id=Harness.next_domain)
        Harness.next_domain += 1
        self.ex = SingleThreadedExecutor()
        self.server = rclpy.create_node('gui_test_task')
        self.active=False; self.enabled=True; self.accept=True; self.goals=[]; self.cancels=0
        self.pub=self.server.create_publisher(TaskStatus,'/task/status',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer=self.server.create_timer(.05,self.status)
        self.action=ActionServer(self.server,RunTask,'/task/run',execute_callback=self.execute,
            goal_callback=lambda _: GoalResponse.ACCEPT if self.accept else GoalResponse.REJECT,
            cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.ready_goals=0
        self.ready_action=ActionServer(self.server,MoveToReady,'/task/move_to_ready',
            execute_callback=self.execute_ready,cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.gui=TaskGui(parameter_overrides=[Parameter('port',value=0)])
        self.ex.add_node(self.server); self.ex.add_node(self.gui)
        self.url=f'http://127.0.0.1:{self.gui.http.server_address[1]}'

    def status(self):
        if self.enabled:
            self.pub.publish(TaskStatus(state='Approach' if self.active else 'Idle',active=self.active))

    async def pause(self, seconds=.03):
        f=Future()
        timer=self.server.create_timer(seconds,lambda: f.set_result(None) if not f.done() else None)
        try: await f
        finally: self.server.destroy_timer(timer)

    async def execute(self, goal):
        self.active=True; self.goals.append(goal.request.target_id)
        while not goal.is_cancel_requested: await self.pause()
        self.cancels+=1
        await self.pause(.25)  # Cancel ACK precedes terminal result.
        goal.canceled(); self.active=False
        return RunTask.Result(success=False,reason='canceled')

    async def execute_ready(self, goal):
        assert not self.active  # Preparation must wait for the old task to terminate.
        self.active=True;self.ready_goals+=1
        await self.pause(.25)
        self.active=False
        if goal.is_cancel_requested:
            goal.canceled();return MoveToReady.Result(success=False,reason='canceled')
        goal.succeed();return MoveToReady.Result(success=True,reason='ready')

    def spin(self, predicate, timeout=5):
        end=time.monotonic()+timeout
        while not predicate():
            assert time.monotonic()<end, self.gui.snapshot()
            self.ex.spin_once(timeout_sec=.01)

    def request(self, command=None, target='', token=None, origin=None):
        body=None if command is None else json.dumps(dict(command=command,target_id=target)).encode()
        headers={'Authorization':'Bearer '+(token or self.gui.token)}
        if origin:headers['Origin']=origin
        req=Request(self.url+('/api/state' if body is None else '/api/command'),data=body,headers=headers)
        with urlopen(req,timeout=2) as r:return json.load(r)

    def close(self):
        if self.gui.goal_handle:
            self.gui.command('stop','');self.spin(lambda:self.gui.goal_handle is None)
        self.ex.shutdown();self.gui.destroy_node();self.server.destroy_node();rclpy.shutdown()


def test_http_action_start_stop_restart_and_loss():
    h=Harness()
    try:
        h.spin(lambda:h.gui.snapshot().get('connected'))
        with pytest.raises(HTTPError) as e:h.request(token='wrong')
        assert e.value.code==401
        with pytest.raises(HTTPError) as e:h.request('start',origin='http://other.example')
        assert e.value.code==403
        with pytest.raises(HTTPError) as e:h.request('jog')
        assert e.value.code==400
        h.request('start','bread_3');h.spin(lambda:len(h.goals)==1)
        h.request('start','duplicate')
        h.spin(lambda:'종료 확인' in h.gui.message)
        assert h.goals==['bread_3']
        h.request('restart','bread_4');h.spin(lambda:h.cancels==1)
        assert h.goals==['bread_3']  # ACK is not terminal completion.
        h.spin(lambda:len(h.goals)==2)
        assert h.goals==['bread_3','bread_4']
        h.request('stop');h.spin(lambda:h.gui.goal_handle is None and not h.gui.sending)
        h.spin(lambda:h.gui.await_idle_after is None)
        assert h.cancels==2
        h.accept=False;h.request('start');h.spin(lambda:'거부' in h.gui.message)
        assert not h.gui.sending and h.gui.goal_handle is None
        h.accept=True
        # Stale status disables new starts even if the Action server is discoverable.
        h.enabled=False;h.spin(lambda:not h.gui.snapshot()['connected'],timeout=4)
        h.request('start');h.spin(lambda:'끊겨' in h.gui.message)
        assert len(h.goals)==2
        h.enabled=True;h.spin(lambda:h.gui.snapshot()['connected'])
        # Never take over a task owned by another Action client.
        h.active=True;h.spin(lambda:h.gui.status.get('active'))
        h.request('restart');h.spin(lambda:'종료 확인' in h.gui.message)
        assert len(h.goals)==2
        h.active=False
    finally:h.close()


def test_stop_during_goal_admission_cancels_late_acceptance():
    h=Harness()
    try:
        h.spin(lambda:h.gui.connected())
        h.gui.command('start','early')
        assert h.gui.sending
        h.gui.command('stop','')
        h.spin(lambda:h.cancels==1)
        h.spin(lambda:h.gui.goal_handle is None and not h.gui.sending)
        assert h.goals==['early']
    finally:h.close()


def test_connection_loss_clears_restart_reservation():
    h=Harness()
    try:
        h.spin(lambda:h.gui.connected())
        h.gui.command('start','original');h.spin(lambda:len(h.goals)==1)
        h.gui.command('restart','replacement')
        h.enabled=False;h.gui.received=time.monotonic()-3
        h.gui.tick()
        assert h.gui.restart_target is None
        h.spin(lambda:h.gui.goal_handle is None and not h.gui.sending and h.gui.result is not None)
        h.enabled=True;h.spin(lambda:h.gui.await_idle_after is None and h.gui.connected())
        assert h.goals==['original']
    finally:h.close()


def test_stage_selection_is_not_an_http_control():
    h=Harness()
    try:
        h.spin(lambda:h.gui.connected())
        body=json.dumps(dict(command='start',start_stage='Place')).encode()
        req=Request(h.url+'/api/command',data=body,headers={'Authorization':'Bearer '+h.gui.token})
        with pytest.raises(HTTPError) as e:urlopen(req,timeout=2)
        assert e.value.code==400 and not h.goals
    finally:h.close()


def test_ready_waits_for_task_termination_without_starting_another_task():
    h=Harness()
    try:
        h.spin(lambda:h.gui.connected() and h.gui.ready_client.server_is_ready())
        h.request('start','bag');h.spin(lambda:len(h.goals)==1)
        h.request('ready');h.spin(lambda:h.cancels==1)
        assert h.ready_goals==0  # Cancel acknowledgement is not a stopped robot.
        h.spin(lambda:h.ready_goals==1)
        h.spin(lambda:h.gui.result is not None and h.gui.result['reason']=='ready')
        h.spin(lambda:h.gui.await_idle_after is None)
        assert h.goals==['bag'] and not h.active
        h.active=True;h.spin(lambda:h.gui.status.get('active'))
        h.request('ready');h.spin(lambda:'종료 확인' in h.gui.message)
        assert h.ready_goals==1
        h.active=False
    finally:h.close()


def test_ready_reservation_clears_on_loss_and_ready_itself_can_be_canceled():
    h=Harness()
    try:
        h.spin(lambda:h.gui.connected() and h.gui.ready_client.server_is_ready())
        h.gui.command('start','bag');h.spin(lambda:len(h.goals)==1)
        h.gui.command('ready','')
        h.enabled=False;h.gui.received=time.monotonic()-3;h.gui.tick()
        assert not h.gui.ready_pending
        h.spin(lambda:h.gui.goal_handle is None and not h.gui.sending and h.gui.result is not None)
        h.enabled=True;h.spin(lambda:h.gui.await_idle_after is None and h.gui.connected())
        assert h.ready_goals==0
        h.request('ready');h.spin(lambda:h.ready_goals==1)
        h.request('stop');h.spin(lambda:h.gui.result is not None and h.gui.result['reason']=='canceled')
        assert h.goals==['bag']
    finally:h.close()
