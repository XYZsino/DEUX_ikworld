"""Synthetic BT sequencing through real ROS child actions; no hardware/planner commands.

Grasp replay is separately exercised by test_grid_grasp_execution_ros --lookup-bt.
Here child results and post-grasp stages are faked to verify BT transitions/failures.
"""
import os
if os.environ.get('ROS_DOMAIN_ID') in (None,'0','12'):raise RuntimeError('isolated ROS domain required')
import time
from types import SimpleNamespace
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import SetBool,Trigger
from deux_task_msgs.action import LookupGrasp,MoveToReady
from deux_task_msgs.msg import VisualEvidence,BarcodeTarget,BarcodeRead,ScanSession
from deux_task_runtime.common import pause,ros_time
from deux_task_core.placement import stages
from lookup_bt_node import LookupTaskNode,RecoverableCycleError,TransientHandFeedbackError


def main():
    rclpy.init(args=['--ros-args','-p','config_file:=src/deux_task_bringup/config/installation.yaml','-p','parallel_reader_retreat:=false'])
    node=LookupTaskNode();fake=Node('lookup_bt_synthetic');fake.group=ReentrantCallbackGroup()
    executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(fake)
    events=[];mode=['normal'];fault=[''];cancelled=[];requests=[];health_good=[True]
    def hold(req,res):events.append(('hold',req.data));res.success=True;return res
    from deux_task_runtime.common import LATCHED
    scanner=fake.create_publisher(BarcodeRead,'/task/barcode_read',10)
    def session(msg):
        if not msg.active or fault[0]!='read-success':return
        result=BarcodeRead(value='synthetic-code',session_id=msg.session_id,track_id=msg.track_id)
        result.header.stamp=ros_time(node.now());scanner.publish(result)
    subscription=fake.create_subscription(ScanSession,'/task/scan_session',session,LATCHED,callback_group=fake.group)
    def health(req,res):res.success=health_good[0];res.message='fake healthy' if health_good[0] else 'hand feedback unavailable';return res
    health_service=fake.create_service(Trigger,'/task/execution_health',health,callback_group=fake.group)
    service=fake.create_service(SetBool,'/task/lookup_perception_hold',hold,callback_group=fake.group)
    async def ready(goal):
        events.append('prepare');goal.succeed();return MoveToReady.Result(success=True,reason='fake ready')
    async def grasp(goal):
        events.append('grasp');requests.append(goal.request.request_id)
        if fault[0]=='cancel':
            while not goal.is_cancel_requested:await pause(fake,.01)
            cancelled.append(True);goal.canceled();return LookupGrasp.Result(success=False,reason='canceled')
        result=LookupGrasp.Result(success=True,path_completed=True,request_id=goal.request.request_id,
            target_id='synthetic-target',cell_id='synthetic-cell',grasp_mode=mode[0])
        result.contact_grasp.header.frame_id='base_link'
        if fault[0]=='lookup' or (fault[0]=='second-cycle-fail' and node.repeat_cycle==2):result.request_id='wrong-request'
        goal.succeed();return result
    from rclpy.action import CancelResponse
    servers=[ActionServer(fake,MoveToReady,'/task/lookup_prepare',ready,callback_group=fake.group),
             ActionServer(fake,LookupGrasp,'/task/lookup_grasp',grasp,callback_group=fake.group,cancel_callback=lambda _:CancelResponse.ACCEPT)]
    async def stage(name,goal,version):
        events.append(name)
        if fault[0]=='feedback-once' and name=='ScanPose' and node.repeat_cycle==1 and not node.recovery_active:
            raise TransientHandFeedbackError('ScanPose [HAND_FEEDBACK_STALE] fake gap')
        if fault[0]=='align-plan-once' and name=='AlignBarcode' and node.repeat_cycle==1:raise RecoverableCycleError('fake IK plan failed')
        if fault[0]=='recovery-fail':
            if name=='AlignBarcode':raise RecoverableCycleError('fake IK plan failed')
            if node.recovery_active and name=='ClearReader':raise ValueError('fake recovery controller failed')
        if fault[0]=='stop-repeat' and name=='ScanPose':
            response=node.set_repeat(SetBool.Request(data=False),SetBool.Response());assert response.success
        if fault[0]=='scan' and name=='ScanPose':raise ValueError('fake scan planning failed')
    async def observed(predicate,goal,version):
        if fault[0]=='barcode':raise TimeoutError('fake barcode absent')
        stamp=ros_time(node.now())
        node.visual=VisualEvidence(target_id=node.target_id,request_id=node.perception_request.request_id,
            barcode_visible=True,barcode_track_id='label',sequence=1)
        node.visual.header.frame_id='base_link';node.visual.header.stamp=stamp
        node.label=BarcodeTarget(target_id=node.target_id,request_id=node.perception_request.request_id,
            barcode_track_id='label',sequence=1)
        node.label.header.frame_id='base_link';node.label.header.stamp=stamp
        assert predicate(),'fake observation did not satisfy actual BT predicate'
    def guard(goal,version):
        if goal.is_cancel_requested or node.estop:raise InterruptedError('operator canceled / e-stop active')
    node.stage=stage;node.await_condition=observed;node.guard=guard
    node.config['skip_visual_pick_verification']=True;node.config['skip_barcode_completion']=True
    def spin_until(predicate,timeout=5.):
        end=time.monotonic()+timeout
        while not predicate() and time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        assert predicate(),'synthetic BT timeout'
    spin_until(lambda:node.lookup_client.server_is_ready() and node.prepare_client.server_is_ready() and node.perception_hold.service_is_ready())
    goal=SimpleNamespace(request=SimpleNamespace(target_id=''),is_cancel_requested=False,is_active=False)
    try:
        for selected in ('normal','transverse_axis_locked'):
            mode[0]=selected;events.clear();task=executor.create_task(node.execute(goal));spin_until(task.done)
            result=task.result();assert result.success
            expected=['prepare','grasp','ScanPose','AlignBarcode',*stages(node.config)]
            assert [e for e in events if isinstance(e,str)]==expected,events
            assert events.index(('hold',True))<events.index('ScanPose'),events
            assert events[-1]==('hold',False),events
            assert not node.physical_pick_verified and 'unverified' in result.reason
        node.config['skip_barcode_completion']=False;node.config['scan_timeout']=.1
        for read_mode in ('read-success','read-timeout'):
            fault[0]=read_mode;events.clear();task=executor.create_task(node.execute(goal));spin_until(task.done)
            if read_mode=='read-success':
                assert task.result().barcode=='synthetic-code' and 'PlaceApproach' in events
            else:
                assert not task.result().success and node.repeat_failed==1 and node.repeat_recovered==1
                assert 'PlaceApproach' in events and events.count('AlignBarcode')==3,events
        node.config['skip_barcode_completion']=True
        for failure in ('lookup','scan','barcode'):
            fault[0]=failure;events.clear();task=executor.create_task(node.execute(goal));spin_until(task.done)
            try:task.result();raise AssertionError('expected BT failure')
            except (ValueError,TimeoutError):pass
            assert 'Release' not in events and 'Place' not in events,events
            if failure=='lookup':assert 'ScanPose' not in events
            assert events[-1]==('hold',False),events
        fault[0]='cancel';events.clear();task=executor.create_task(node.execute(goal))
        spin_until(lambda:'grasp' in events);goal.is_cancel_requested=True;spin_until(task.done)
        try:task.result();raise AssertionError('expected cancel')
        except InterruptedError:pass
        spin_until(lambda:bool(cancelled));assert 'ScanPose' not in events and events[-1]==('hold',False)
        from rclpy.parameter import Parameter
        goal.is_cancel_requested=False;fault[0]='';events.clear();requests.clear()
        node.repeat_enabled=True;node.set_parameters([Parameter('repeat_max_cycles',value=2)])
        task=executor.create_task(node.execute(goal));spin_until(task.done)
        assert task.result().success and node.repeat_completed==2 and events.count('prepare')==2
        assert len(requests)==len(set(requests))==2,'fresh lookup request IDs required each cycle'
        node.set_parameters([Parameter('repeat_max_cycles',value=0)])
        node.repeat_enabled=True;fault[0]='stop-repeat';events.clear()
        task=executor.create_task(node.execute(goal));spin_until(task.done)
        assert task.result().success and node.repeat_completed==1 and events.count('prepare')==1
        assert 'PlaceApproach' in events,'disable repeat must finish current cycle'
        node.repeat_enabled=True;fault[0]='';events.clear()
        task=executor.create_task(node.execute(goal));spin_until(lambda:node.state=='RepeatReady')
        response=node.set_repeat(SetBool.Request(data=False),SetBool.Response());assert response.success
        spin_until(task.done);assert task.result().success and node.state=='Succeeded'
        assert events.count('prepare')==1,'between-cycle disable started another cycle'
        node.repeat_enabled=True;fault[0]='second-cycle-fail';events.clear()
        task=executor.create_task(node.execute(goal));spin_until(task.done)
        try:task.result();raise AssertionError('expected second-cycle failure')
        except ValueError:pass
        assert node.repeat_completed==1 and events.count('prepare')==2 and events.count('ScanPose')==1
        assert not node.repeat_enabled,'failure must disable repeat'
        node.repeat_enabled=True;fault[0]='';events.clear()
        task=executor.create_task(node.execute(goal));spin_until(lambda:node.state=='RepeatReady')
        goal.is_cancel_requested=True;spin_until(task.done)
        try:task.result();raise AssertionError('expected between-cycle cancel')
        except InterruptedError:pass
        assert events.count('prepare')==1 and not node.repeat_enabled
        goal.is_cancel_requested=False;node.repeat_enabled=True;fault[0]='align-plan-once';events.clear()
        node.set_parameters([Parameter('repeat_max_cycles',value=1)])
        task=executor.create_task(node.execute(goal));spin_until(task.done)
        assert task.result().success and node.repeat_cycle==2 and node.repeat_completed==1
        assert node.repeat_failed==1 and node.repeat_recovered==1 and events.count('PlaceApproach')==2
        positions=[i for i,e in enumerate(events) if e=='prepare']
        assert events.index('PlaceApproach')<positions[1],'next prepare ran before failed object was released'
        node.repeat_enabled=True;fault[0]='recovery-fail';events.clear()
        task=executor.create_task(node.execute(goal));spin_until(task.done)
        try:task.result();raise AssertionError('expected recovery failure')
        except ValueError:pass
        assert node.repeat_failed==1 and node.repeat_recovered==0 and events.count('prepare')==1
        assert not node.repeat_enabled
        node.repeat_enabled=False;node.recover_before_next=True;fault[0]='';events.clear()
        task=executor.create_task(node.execute(goal));spin_until(task.done)
        assert task.result().success and node.repeat_recovered==1 and not node.recover_before_next
        assert events.index('PlaceApproach')<events.index('prepare'),'previous failed loaded cycle not cleared first'
        node.repeat_enabled=True;fault[0]='feedback-once';events.clear();health_good[0]=False
        task=executor.create_task(node.execute(goal));spin_until(lambda:node.state=='WaitHandFeedback')
        end=time.monotonic()+.2
        while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
        assert 'PlaceApproach' not in events,'recovery moved without healthy hand feedback'
        health_good[0]=True;spin_until(task.done,8.)
        assert task.result().success and node.repeat_failed==1 and node.repeat_recovered==1 and node.repeat_completed==1
        node.repeat_enabled=True;fault[0]='feedback-once';events.clear();health_good[0]=False
        task=executor.create_task(node.execute(goal));spin_until(lambda:node.state=='WaitHandFeedback')
        goal.is_cancel_requested=True;spin_until(task.done)
        try:task.result();raise AssertionError('expected cancellation during feedback wait')
        except InterruptedError:pass
        assert 'PlaceApproach' not in events and not node.repeat_enabled
        goal.is_cancel_requested=False;health_good[0]=True
        from unittest.mock import patch
        from deux_task_runtime.head_task_node import HeadTaskNode
        async def planning_failure(*args,**kwargs):raise ValueError('fake backend failure')
        async def classify(client):
            try:await node.call(client,SimpleNamespace(stage='AlignBarcode'),goal,node.config_version,1.)
            except Exception as error:return type(error)
            raise AssertionError('expected injected call failure')
        with patch.object(HeadTaskNode,'call',planning_failure):
            task=executor.create_task(classify(node.planner));spin_until(task.done)
            assert task.result() is RecoverableCycleError
            task=executor.create_task(classify(node.executor_client));spin_until(task.done)
            assert task.result() is ValueError,'controller failure classified recoverable'
        print('PASS: transient feedback waits without motion, stable health -> recovery/next object; persistent feedback wait remains cancelable')
        print('PASS: failed barcode plan -> fixed-bin release/retreat -> next object; recovery failure stops; prior-session recovery; controller errors remain fatal')
        print('PASS: repeat two cycles/fresh IDs; finish-current disable; second-cycle failure; between-cycle cancel')
        print('PASS: two sequential angle classes, correlated handoff, lease ownership, lookup/scan/barcode failure boundaries, barcode read/three bounded timeouts, operator cancel')
    finally:
        for server in servers:server.destroy()
        node.destroy_node();fake.destroy_node();executor.shutdown();rclpy.shutdown()


if __name__=='__main__':main()
