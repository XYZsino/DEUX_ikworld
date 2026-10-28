"""The BT must forward the selected handoff, rather than silently scheduling anew."""
import asyncio
from types import SimpleNamespace
import pytest
pytest.importorskip('rclpy')
from deux_task_msgs.msg import MotionPlan,StateObservation
from trajectory_msgs.msg import JointTrajectoryPoint
from deux_task_runtime.task_node import TaskNode
from deux_task_runtime.common import ros_time,seconds
from builtin_interfaces.msg import Duration


@pytest.mark.parametrize('finished',[True,False])
def test_completed_child_is_not_canceled_but_interrupted_child_is(monkeypatch,finished):
    import deux_task_runtime.task_node as module
    from action_msgs.msg import GoalStatus
    canceled=[]
    result=SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,result=SimpleNamespace(success=True))
    future=SimpleNamespace(done=lambda:finished,result=lambda:result)
    handle=SimpleNamespace(accepted=True,get_result_async=lambda:future)
    send=SimpleNamespace(done=lambda:True,result=lambda:handle)
    client=SimpleNamespace(send_goal_async=lambda *a,**kw:send)
    async def wait(*a):return handle
    async def cancel(node,handles):canceled.extend(handles)
    def guard(*a):raise InterruptedError('parent canceled')
    monkeypatch.setattr(module,'wait',wait);monkeypatch.setattr(module,'cancel_children',cancel)
    node=SimpleNamespace(guard=guard,child=None,state='Approach')
    if finished:asyncio.run(TaskNode.call(node,client,SimpleNamespace(),None,'scene',1.))
    else:
        with pytest.raises(InterruptedError):asyncio.run(TaskNode.call(node,client,SimpleNamespace(),None,'scene',1.))
    assert canceled==([] if finished else [handle])


def test_bt_carries_absolute_grasp_clock_and_phases_between_actions():
    requests=[]
    planner=object();executor=object()
    plan=MotionPlan(ik_backend='pyroki',grasp_candidate_id='yaw_minus_24')
    plan.grasp_start_at=ros_time(123.45);plan.grasp_phase_durations=[0.,1.1,1.2]
    plan.trajectory.points=[JointTrajectoryPoint(time_from_start=ros_time(2.,Duration))]
    async def call(client,request,*args):
        if client is planner:
            requests.append(request);return SimpleNamespace(plan=plan)
        return SimpleNamespace(reason='executed')
    node=SimpleNamespace(guard=lambda *a:None,publish_status=lambda:None,planner=planner,
        executor_client=executor,call=call,target_id='bag',label=None,ik_calls=0,
        planning_observation=lambda name:StateObservation())
    asyncio.run(TaskNode.stage(node,'Approach',None,'scene'))
    asyncio.run(TaskNode.stage(node,'TrackGrasp',None,'scene'))
    assert seconds(requests[0].grasp_start_at)==0
    assert seconds(requests[1].grasp_start_at)==pytest.approx(123.45)
    assert list(requests[1].grasp_phase_durations)==[0.,1.1,1.2]
    assert requests[1].grasp_candidate_id=='yaw_minus_24'


@pytest.mark.parametrize('delays,search_count', [([1.,.1],1),([6.,1.,.1],2),([6.,8.,12.],3)])
def test_planner_preserves_reserved_start_and_researches_after_overrun(monkeypatch,delays,search_count):
    """Exercise the real action callback across delayed workers and fresh stamps."""
    import numpy as np
    from copy import deepcopy
    from deux_task_msgs.action import PlanMotion
    from deux_task_msgs.msg import ObjectDetection
    from deux_task_core.planning_latency import PlanningLatency
    import deux_task_runtime.motion_planner_node as module
    clock=[100.];requests=[];reports=[];steps=iter(delays)
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock[0]))
    config=dict(perception_policy='head_rgbd',moving_source=True,base_frame='base_link',
        model_id='test',synthetic=True,start_tolerance=.15,max_plan_age=3.,
        conveyor_tracking=dict(start_lead=.8,candidate_search=dict(enabled=True,
            selection_start_lead_s=1.,time_search=dict(enabled=True))))
    obs=StateObservation();obs.objects.header.stamp=ros_time(99.8)
    obj=ObjectDetection(track_id='bag',pose_filtered=True,velocity_valid=True)
    obj.pose.pose.orientation.w=1.;obj.velocity.linear.y=-.09
    latency=PlanningLatency(config)
    node=SimpleNamespace(config=config,config_version='scene',latency_scene_version='scene',
        planning_latency=latency,now=lambda:clock[0],measured=lambda:np.zeros(28),
        target=lambda *a,**k:obj,grasp_pose_age=lambda s:2.,observation=deepcopy(obs),
        candidate_pub=SimpleNamespace(publish=lambda msg:reports.append(msg.data)),
        get_logger=lambda:SimpleNamespace(info=lambda *a:None,warning=lambda *a:None),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:ros_time(clock[0]))))
    node.sample_planning_clock=lambda:latency.sample(clock[0],clock[0])
    class Work:
        def __init__(self,args):self.args=args;self.finished=False
        def done(self):
            if not self.finished:
                clock[0]+=next(steps);node.observation.objects.header.stamp=ros_time(clock[0]-.1)
                self.finished=True
            return True
        def result(self):
            lead=self.args[6][1];timing=self.args[8]
            if timing is None:timing=dict(arrival_lead_s=lead+2.,grasp_start_lead_s=lead+3.,phase_durations=[0.,.8,1.])
            duration=timing['arrival_lead_s']-lead
            # Regression: refreshing perception must not squeeze or extend the
            # previously checked approach while preserving its arrival clock.
            assert duration==pytest.approx(2.)
            return dict(candidate_id='nominal',timing=timing,ik_backend='pyroki',
                ik_calls=1,minimum_clearance=.1,times=np.array([0.,duration]),path=np.zeros((2,28)),
                candidate_report=dict(scope='test',candidates=[],selected_candidate_id='nominal'))
    def submit(fn,*args):requests.append(args);return Work(args)
    node.pool=SimpleNamespace(submit=submit)
    goal=SimpleNamespace(request=PlanMotion.Goal(stage='Approach',target_id='bag',observation=obs),is_cancel_requested=False)
    if search_count==3:
        with pytest.raises(ValueError,match='budget exhausted'):
            asyncio.run(module.MotionPlannerNode.execute(node,goal))
        assert len(requests)==3
        return
    result=asyncio.run(module.MotionPlannerNode.execute(node,goal))
    assert result.success
    assert sum(r[8] is None for r in requests)==search_count
    assert len(requests)==search_count+1
    assert seconds(result.plan.trajectory.header.stamp)-clock[0]>=latency.config['dispatch_lead_s']
    assert seconds(result.plan.grasp_start_at)-seconds(result.plan.trajectory.header.stamp)==pytest.approx(3.)
    if search_count==2:assert any('expired_research_required' in r for r in reports)



@pytest.mark.parametrize('occluded,shift',[(False,False),(False,True)])
def test_preflight_keeps_path_or_researches_on_large_change(monkeypatch,occluded,shift):
    import numpy as np
    from copy import deepcopy
    from deux_task_msgs.action import PlanMotion
    from deux_task_msgs.msg import ObjectDetection
    from deux_task_core.planning_latency import PlanningLatency
    import deux_task_runtime.motion_planner_node as module
    clock=[100.];requests=[];reports=[]
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=lambda:clock[0]))
    c=dict(perception_policy='head_rgbd',moving_source=True,base_frame='base_link',model_id='test',
        synthetic=True,start_tolerance=.15,max_plan_age=3.,max_object_age=.5,max_target_drift=.04,
        conveyor_tracking=dict(start_lead=.8,
            candidate_search=dict(enabled=True,selection_start_lead_s=1.,time_search=dict(enabled=True,reuse_validated_path=True))))
    obs=StateObservation();obs.objects_valid=True;obs.objects.header.stamp=ros_time(99.8)
    obj=ObjectDetection(track_id='bag',pose_filtered=True,velocity_valid=True)
    obj.pose.pose.orientation.w=1.;obj.velocity.linear.y=-.09;obs.objects.detections=[obj]
    latency=PlanningLatency(c)
    node=SimpleNamespace(config=c,config_version='scene',latency_scene_version='scene',planning_latency=latency,
        now=lambda:clock[0],measured=lambda:np.zeros(28),grasp_pose_age=lambda s:2.,observation=deepcopy(obs),
        target=lambda key,state=None,**kw:(state or node.observation).objects.detections[0],
        candidate_pub=SimpleNamespace(publish=lambda msg:reports.append(msg.data)),
        get_logger=lambda:SimpleNamespace(info=lambda *a:None,warning=lambda *a:None),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:ros_time(clock[0]))))
    node.sample_planning_clock=lambda:latency.sample(clock[0],clock[0])
    class Work:
        def __init__(self,args):self.args=args
        def done(self):
            clock[0]+=.1;node.observation.objects.header.stamp=ros_time(clock[0]-.1)
            node.observation.objects.detections[0].pose.pose.position.y=-.09*(clock[0]-.1-99.8)
            node.observation.objects.detections[0].pose.pose.position.x=.06 if shift else 0.
            node.observation.objects_valid=not occluded
            return True
        def result(self):
            lead=self.args[6][1]
            return dict(candidate_id='nominal',timing=dict(arrival_lead_s=lead+1.,grasp_start_lead_s=lead,phase_durations=[1.,.5,.5]),
                ik_backend='pyroki',ik_calls=1,minimum_clearance=.1,times=np.array([0.,2.]),path=np.zeros((2,28)),
                phase_end_times=np.array([1.,1.5,2.]),candidate_report=dict(scope='search',candidates=[],selected_candidate_id='nominal'))
    def submit(fn,*args):requests.append(args);return Work(args)
    node.pool=SimpleNamespace(submit=submit)
    goal=SimpleNamespace(request=PlanMotion.Goal(stage='Approach',target_id='bag',observation=obs),is_cancel_requested=False)
    result=asyncio.run(module.MotionPlannerNode.execute(node,goal))
    assert len(requests)==(2 if shift else 1)
    assert all(args[8] is None for args in requests)  # Only searches, never repeated IK validation.
    assert seconds(result.plan.target_acquired_at)==pytest.approx(100. if shift else 99.8)
    assert result.plan.target_pose.position.y==pytest.approx(-.018 if shift else 0.)
    assert list(result.plan.trajectory.points[-1].positions)==[0.]*28
    assert any('"ik_recomputed": false' in r for r in reports)
