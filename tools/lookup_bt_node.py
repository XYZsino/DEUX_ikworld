"""Full BT: audited ready -> lookup grasp/lift -> scan -> barcode -> bin/retreat."""
from uuid import uuid4
import json,time
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_names_and_types
from std_msgs.msg import Bool,Header,String
from std_srvs.srv import SetBool,Trigger
from deux_task_msgs.action import LookupGrasp,MoveToReady,RunTask
from deux_task_msgs.msg import ScanSession
from deux_task_runtime.head_task_node import HeadTaskNode
from deux_task_runtime.behavior_tree import Leaf,Sequence,Retry,Failure
from deux_task_runtime.common import run,current,seconds,ros_time,pause,cancel_children,LATCHED,ExclusiveAction
from deux_task_core.placement import enabled,stages
from lookup_bt_support import validate_handoff
from scan_bbox_gate import matching_boxes


class ScanObjectMissing(ValueError):
    """No sufficiently large bbox at scan: return directly to ready."""


class RecoverableCycleError(ValueError):
    """Post-scan planning/read failure; no failed controller motion is retried."""


class TransientHandFeedbackError(RecoverableCycleError):
    pass


class LookupTaskNode(HeadTaskNode):
    def __init__(self,**kwargs):
        super().__init__(**kwargs)
        self.workflow='lookup_pick_scan_place'
        self.declare_parameter('enable_execution',False)
        self.declare_parameter('scan_speed_scale',.5)
        if not float(self.config.get('scan_motion_scale',1.))<=float(self.get_parameter('scan_speed_scale').value)<=1.:
            raise ValueError('scan_speed_scale must be between configured scan scale and 1')
        self.parallel_timing={}
        self.declare_parameter('scan_bbox_wait_s',1.0)
        self.declare_parameter('scan_bag_min_area_px2',5174.0)
        self.declare_parameter('scan_barcode_min_area_px2',300.0)
        self.declare_parameter('scan_presence_roi_file',str(Path(__file__).resolve().parents[1]/'calibration/via_presence_roi.json'))
        self.declare_parameter('scan_rgb_info_topic','/zed/color/camera_info')
        self.scan_camera_info=None
        self.declare_parameter('via_presence_min_frames',5)
        self.declare_parameter('via_presence_min_confidence',.5)
        if not .5<=float(self.get_parameter('via_presence_min_confidence').value)<=1.:raise ValueError('via presence confidence must be within [0.5,1]')
        self.declare_parameter('via_depth_tolerance_m',.15)
        if int(self.get_parameter('via_presence_min_frames').value)<5:raise ValueError('via presence requires at least 5 frames')
        if not 0<float(self.get_parameter('via_depth_tolerance_m').value)<=.3:raise ValueError('invalid via depth tolerance')
        from via_depth_gate import ViaDepthGate
        self.via_depth=ViaDepthGate(self)
        from sensor_msgs.msg import CameraInfo
        self.scan_bbox_evidence={};self.scan_bbox_context=None
        self.via_timings=json.loads((Path(__file__).resolve().parents[1]/'artifacts/scan-speed-search/library-validation.json').read_text())['accepted']
        self.shutdown_requested=False
        from rclpy.qos import qos_profile_sensor_data
        self.scan_info_subscription=self.create_subscription(CameraInfo,self.get_parameter('scan_rgb_info_topic').value,
            lambda msg:setattr(self,'scan_camera_info',msg),qos_profile_sensor_data)
        from deux_task_msgs.msg import DetectionArray
        self.scan_bbox_subscription=self.create_subscription(DetectionArray,'/task/detections',
            self.receive_scan_boxes,qos_profile_sensor_data)
        if any(float(self.get_parameter(p).value)<=0 for p in
               ('scan_bbox_wait_s','scan_bag_min_area_px2','scan_barcode_min_area_px2')):
            raise ValueError('scan bbox timeout and minimum areas must be positive')
        self.declare_parameter('parallel_reader_retreat',True)
        self.declare_parameter('dataset_reading_pose_file','')
        from deux_task_runtime.dataset_reading_pose import load_pose
        self.dataset_reading_pose=load_pose(self.get_parameter('dataset_reading_pose_file').value)
        self.subscribe_joints()
        self.scan_timing={}
        self.transfer_pool=ThreadPoolExecutor(max_workers=1)
        self.execution_health=self.create_client(Trigger,'/task/execution_health',callback_group=self.group)
        self.declare_parameter('repeat_execution',False)
        self.declare_parameter('repeat_max_cycles',0)
        self.repeat_enabled=bool(self.get_parameter('repeat_execution').value)
        self.repeat_failed=0;self.repeat_recovered=0;self.repeat_last_error='';self.recovery_active=False;self.recover_before_next=False
        self.repeat_cycle=0;self.repeat_completed=0;self.repeat_session_id='';self.repeat_target_id=''
        self.repeat_pub=self.create_publisher(String,'/task/repeat_status',LATCHED)
        self.repeat_service=self.create_service(SetBool,'/task/repeat_execution',self.set_repeat,callback_group=self.group)
        self.create_timer(.5,self.publish_repeat_status)
        self.create_service(SetBool,'/task/recover_before_next',self.set_recover_before_next,callback_group=self.group)
        self.soft_stop_client=self.create_client(Trigger,'/task/lookup_soft_stop',callback_group=self.group)
        self.empty_return_client=ActionClient(self,MoveToReady,'/task/lookup_empty_return',callback_group=self.group)
        self.lookup_client=ActionClient(self,LookupGrasp,'/task/lookup_grasp',callback_group=self.group)
        self.prepare_client=ActionClient(self,MoveToReady,'/task/lookup_prepare',callback_group=self.group)
        self.attention_client=ActionClient(self,MoveToReady,'/task/lookup_attention',callback_group=self.group)
        self.attention_action=ExclusiveAction(self,MoveToReady,'/task/return_to_attention',
            self.return_to_attention,self.validate_attention,gate=self.action.gate)
        self.perception_hold=self.create_client(SetBool,'/task/lookup_perception_hold',callback_group=self.group)
        self.estop=False;self.lookup_delegated=False;self.lookup_committed=False
        self.last_lookup_result=None;self.physical_pick_verified=False
        self.bt_goal=None;self.bt_events=[]
        self.declare_parameter('bt_output_directory',str(Path(__file__).resolve().parents[1]/'.setup/lookup-bt'))
        self.create_subscription(Bool,'/deux/estop',lambda m:setattr(self,'estop',m.data),10)

    def receive_scan_boxes(self,msg):
        context=self.scan_bbox_context
        if context is None:return
        if time.monotonic()>context.get('deadline_monotonic',float('inf')):return
        if context.get('matched_frames',0)>=context.get('required_frames',5):return
        info=self.scan_camera_info
        if info is None or not info.header.frame_id or info.header.frame_id!=msg.header.frame_id:return
        stamp=seconds(msg.header.stamp)
        identity=(msg.header.frame_id,msg.header.stamp.sec,msg.header.stamp.nanosec)
        if identity in self.scan_seen_frames:return
        self.scan_seen_frames.add(identity)
        if msg.request_id==context['request_id'] and context['arrival_s']<=stamp<=self.now() and self.now()-stamp<=self.config['max_object_age']:
            self.scan_bbox_evidence['fresh_frames']+=1
            self.scan_bbox_evidence['last_boxes']=[dict(class_name=b.class_name,confidence=float(b.confidence),
                xyxy=list(b.xyxy),track_id=b.track_id) for b in msg.boxes]
        matches=matching_boxes(msg,context['request_id'],context['arrival_s'],self.now(),
            self.config['max_object_age'],context['bag_min_area_px2'],context['barcode_min_area_px2'],
            roi=context['roi'],image_size=(info.width,info.height),min_confidence=context['min_confidence'])
        if msg.request_id!=context['request_id'] or not context['arrival_s']<=stamp<=self.now() or self.now()-stamp>self.config['max_object_age']:return
        try:
            matches,depth_evidence=self.via_depth.filter(msg,matches,float(self.get_parameter('via_depth_tolerance_m').value))
            self.scan_bbox_evidence['last_depth']=depth_evidence
        except Exception as error:
            matches=[];self.scan_bbox_evidence['last_depth_error']=str(error)
        if matches and time.monotonic()<=context.get('deadline_monotonic',float('inf')):
            self.scan_bbox_evidence['image_size']=[info.width,info.height]
            self.scan_bbox_evidence['matches']=matches
            self.scan_bbox_evidence['matched_frames']+=1
            self.scan_bbox_evidence['matched_stamps_s'].append(stamp)

    async def scan_bbox_gate(self,goal,version,window_end=None):
        self.set_state('ViaBBoxCheck')
        from deux_task_perception.tracking_roi import read_roi
        roi=read_roi(self.get_parameter('scan_presence_roi_file').value)
        if roi is None:raise ValueError('scan presence ROI is missing')
        self.scan_seen_frames=set()
        self.scan_bbox_evidence=dict(required_frames=int(self.get_parameter('via_presence_min_frames').value),matched_frames=0,matched_stamps_s=[],
            min_confidence=float(self.get_parameter('via_presence_min_confidence').value),
            depth_tolerance_m=float(self.get_parameter('via_depth_tolerance_m').value),arrival_s=self.now(),request_id=self.perception_request.request_id,
            roi=roi,roi_file=self.get_parameter('scan_presence_roi_file').value,
            bag_min_area_px2=float(self.get_parameter('scan_bag_min_area_px2').value),
            barcode_min_area_px2=float(self.get_parameter('scan_barcode_min_area_px2').value),
            wait_s=float(self.get_parameter('scan_bbox_wait_s').value),matches=[],fresh_frames=0,last_boxes=[])
        self.scan_bbox_context=self.scan_bbox_evidence
        deadline=time.monotonic()+self.scan_bbox_evidence['wait_s']
        if window_end is not None:deadline=min(deadline,window_end)
        self.scan_bbox_evidence['deadline_monotonic']=deadline
        self.scan_bbox_evidence['stage']='moving_via'
        self.scan_bbox_evidence['remaining_window_s']=max(0.,deadline-time.monotonic())
        try:
            while True:
                self.guard(goal,version)
                self.scan_bbox_evidence['archive_status']=self.via_depth.recording_status()
                if self.scan_bbox_evidence['matched_frames']>=self.scan_bbox_evidence['required_frames']:
                    self.scan_bbox_evidence['passed']=True
                    self.get_logger().info('scan bbox gate passed: '+json.dumps(self.scan_bbox_evidence))
                    return
                if time.monotonic()>=deadline:break
                await pause(self,.02)
            self.scan_bbox_evidence['passed']=False
            raise ScanObjectMissing('via bbox/depth did not qualify in 5 frames; cancel remaining scan and return to ready')
        finally:self.scan_bbox_context=None

    def set_recover_before_next(self,request,response):
        if self.action.busy or self.ready_action.busy:
            response.success=False;response.message='BT must be idle';return response
        if request.data and (not self.get_parameter('enable_execution').value or not enabled(self.config)
                             or not self.config['placement'].get('fixed_joint_motion',False)):
            response.success=False;response.message='enabled fixed-bin recovery required';return response
        self.recover_before_next=request.data
        response.success=True;response.message='next RunTask first releases at existing bin and retreats' if request.data else 'pending recovery cleared'
        return response

    def set_repeat(self,request,response):
        if request.data and (self.shutdown_requested or self.attention_action.busy):
            response.success=False;response.message='shutdown requested; restart BT before another task';return response
        if request.data and not self.get_parameter('enable_execution').value:
            response.success=False;response.message='preview only';return response
        if request.data and self.action.busy and self.repeat_target_id:
            response.success=False;response.message='repeat requires empty target_id';return response
        self.repeat_enabled=request.data
        response.success=True
        response.message='repeat enabled; RunTask starts execution' if request.data else 'finish current cycle then stop; cancel RunTask for immediate stop'
        self.publish_repeat_status();return response

    def publish_repeat_status(self):
        record=dict(enabled=self.repeat_enabled,active=self.action.busy,shutdown_requested=self.shutdown_requested,
            session_id=self.repeat_session_id,cycle=self.repeat_cycle,completed=self.repeat_completed,
            scan_timing=self.scan_timing,parallel_timing=self.parallel_timing,failed=self.repeat_failed,recovered=self.repeat_recovered,last_error=self.repeat_last_error,recovery_active=self.recovery_active,
            state=self.state,reason=self.reason,stamp_s=self.now())
        self.repeat_pub.publish(String(data=json.dumps(record)))
        try:
            directory=Path(self.get_parameter('bt_output_directory').value);directory.mkdir(parents=True,exist_ok=True)
            (directory/'repeat-status.json').write_text(json.dumps(record,indent=2))
        except OSError as error:self.get_logger().error(f'repeat status write failed: {error}')

    def publish_status(self):
        super().publish_status()
        goal=getattr(self,'bt_goal',None)
        if goal is not None and goal.is_active:
            goal.publish_feedback(RunTask.Feedback(state=self.state,grasp_attempt=self.grasp_attempt,scan_attempt=self.scan_attempt))
        events=getattr(self,'bt_events',None)
        if events is not None and (not events or events[-1]['state']!=self.state):
            events.append(dict(stamp_s=self.now(),state=self.state,reason=self.reason,target_id=self.target_id))

    def validate(self,request):
        if self.shutdown_requested:raise ValueError('shutdown requested; restart BT before another task')
        if not self.get_parameter('enable_execution').value:raise ValueError('lookup BT preview only')
        self.robot_ready()
        if self.repeat_enabled and request.target_id:raise ValueError('repeat requires empty target_id')
        if self.estop:raise ValueError('e-stop active')
        if not self.perception_hold.service_is_ready():raise ValueError('lookup perception bridge unavailable')
        if not all(c.server_is_ready() for c in [self.lookup_client,self.prepare_client,self.planner,self.executor_client]):
            raise ValueError('lookup/prepare/planner/executor unavailable')
        if any(name=='/task/grid_grasp' for name,_ in get_action_names_and_types(self)):
            raise ValueError('standalone grid server conflicts with lookup BT')

    def validate_ready(self,request):
        if self.shutdown_requested:raise ValueError('shutdown requested; only attention return retry is allowed')
        if not self.get_parameter('enable_execution').value:raise ValueError('lookup BT preview only')
        self.robot_ready()
        if self.estop or not self.prepare_client.server_is_ready():raise ValueError('lookup preparation unavailable')

    def validate_attention(self,request):
        if not self.get_parameter('enable_execution').value:raise ValueError('lookup BT preview only')
        self.robot_ready()
        if self.estop:raise ValueError('e-stop active')
        if self.repeat_enabled:raise ValueError('disable repetition before returning to attention')
        if not self.attention_client.server_is_ready():raise ValueError('attention return server unavailable')

    def guard(self,goal,version):
        if self.estop or goal.is_cancel_requested:raise InterruptedError('operator canceled / e-stop active')
        if self.lookup_delegated:
            # The child owns all admission and committed replay diagnostics.
            # Do not cancel its stored path for stale camera/joint/model status.
            return
        super().guard(goal,version)

    async def child_call(self,client,request,goal,version,lookup=False):
        handle=None;terminal=False;pending=None;checkpoint=False;latest={}
        self.lookup_delegated=True
        def feedback(msg):
            latest.update(feedback=msg.feedback,received=time.monotonic())
            self.lookup_committed|=msg.feedback.committed
            self.reason=f'lookup {msg.feedback.state} · {msg.feedback.cell_id}'
            self.publish_status()
        try:
            pending=client.send_goal_async(request,feedback_callback=feedback if lookup else None)
            while not pending.done():self.guard(goal,version);await pause(self,.01)
            handle=pending.result()
            if not handle.accepted:raise ValueError('BT child action rejected')
            self.child=handle;future=handle.get_result_async()
            while not future.done():
                self.guard(goal,version)
                fb=latest.get('feedback')
                if lookup and not checkpoint and fb is not None and fb.committed and fb.cell_id:
                    row=self.via_timings.get(fb.cell_id)
                    if row is None:raise ValueError('missing audited via checkpoint timing')
                    via=row['waypoint_at_s']
                    if fb.elapsed_s>=via-.5:
                        window_end=latest['received']+via+.5-fb.elapsed_s
                        self.target_id=fb.target_id
                        await self.hold_perception(True)
                        self.request_perception('barcode')
                        await self.scan_bbox_gate(goal,version,window_end=window_end)
                        checkpoint=True
                await pause(self,.01)
            response=future.result();terminal=True
            if lookup and response.result.success and not checkpoint:
                raise ValueError('lookup finished without via presence checkpoint')
            if response.status!=4 or not response.result.success:raise ValueError(response.result.reason)
            return response.result
        except ScanObjectMissing:
            # Confirm controller/action termination before the recovery branch may move.
            if handle is not None and not terminal:
                if not self.soft_stop_client.service_is_ready():raise RuntimeError('soft-stop service unavailable; recovery inhibited')
                stop=self.soft_stop_client.call_async(Trigger.Request());until=time.monotonic()+8.
                while not stop.done():
                    self.guard(goal,version)
                    if time.monotonic()>until:raise RuntimeError('soft-stop request timed out; recovery inhibited')
                    await pause(self,.01)
                if not stop.result().success:raise RuntimeError('soft-stop rejected: '+stop.result().message)
                done=handle.get_result_async()
                while not done.done():
                    self.guard(goal,version)
                    if time.monotonic()>until:raise RuntimeError('via stop termination unconfirmed; recovery inhibited')
                    await pause(self,.01)
                terminal=True
                if not all(marker in done.result().result.reason for marker in ('replay_stop_confirmed','soft_stop_completed')):
                    raise RuntimeError('controller stop not confirmed by lookup server; recovery inhibited')
            raise
        finally:
            if handle is not None and not terminal:await cancel_children(self,[handle])
            elif pending is not None and not pending.done():
                def late(f):
                    if f.result().accepted:f.result().cancel_goal_async()
                pending.add_done_callback(late)
            self.child=None;self.lookup_delegated=False

    async def hold_perception(self,held):
        if not self.perception_hold.service_is_ready():raise ValueError('lookup perception bridge unavailable')
        future=self.perception_hold.call_async(SetBool.Request(data=held))
        deadline=time.monotonic()+3.
        while not future.done():
            if time.monotonic()>deadline:raise TimeoutError('perception ownership handshake timed out')
            await pause(self,.01)
        if not future.result().success:raise ValueError(future.result().message)

    async def move_to_ready(self,goal):
        version=self.config_version;self.previous_clock=self.now();self.target_id=''
        self.state='LookupPrepare';self.reason='restore installation left home and saved right ready'
        self.perception_request=None;self.close_session();self.publish_status()
        try:
            result=await self.child_call(self.prepare_client,MoveToReady.Goal(),goal,version)
            self.prepared=True;self.state='Ready';self.reason=result.reason
            return MoveToReady.Result(success=True,reason=result.reason)
        finally:self.publish_status()

    async def return_to_attention(self,goal):
        self.shutdown_requested=True
        version=self.config_version;self.previous_clock=self.now();self.target_id=''
        self.state='ReturnToAttention';self.reason='current -> ready -> reverse simultaneous ready route'
        self.perception_request=None;self.close_session();self.publish_status()
        try:
            result=await self.child_call(self.attention_client,MoveToReady.Goal(),goal,version)
            self.prepared=False;self.state='Attention';self.reason=result.reason
            return result
        except BaseException as error:
            self.state='AttentionReturnFailed';self.reason=str(error);raise
        finally:self.publish_status()

    async def wait_feedback_health(self,goal):
        self.set_state('WaitHandFeedback');stable_since=None
        while True:
            if goal.is_cancel_requested or self.estop:raise InterruptedError('operator canceled / e-stop active')
            if not self.execution_health.service_is_ready():
                self.reason='waiting for execution health service';await pause(self,.1);continue
            future=self.execution_health.call_async(Trigger.Request());deadline=time.monotonic()+1.
            while not future.done() and time.monotonic()<deadline:
                if goal.is_cancel_requested or self.estop:raise InterruptedError('operator canceled / e-stop active')
                await pause(self,.01)
            if not future.done():
                future.cancel();stable_since=None;self.reason='execution health response delayed'
            elif future.result().success:
                if stable_since is None:stable_since=time.monotonic()
                self.reason='feedback healthy; confirming stable reception before recovery'
                if time.monotonic()-stable_since>=1.:return
            else:
                stable_since=None;self.reason=f'waiting without motion: {future.result().message}'
            self.publish_status();await pause(self,.1)

    async def call(self,client,request,goal,version,timeout):
        if client is self.executor_client and getattr(getattr(request,'plan',None),'stage','')=='ScanPose':
            from lookup_scan_timing import retime_scan
            from deux_task_core.model import RobotModel
            request.plan,self.scan_timing=retime_scan(request.plan,self.config,RobotModel(self.config['robot']),self.get_parameter('scan_speed_scale').value)
            self.get_logger().info('ScanPose timing: '+json.dumps(self.scan_timing))
        if client is self.executor_client and (getattr(getattr(request,'plan',None),'stage','') in ('ScanPose','AlignBarcode') or self.recovery_active):
            from lookup_conveyor import validate_conveyor_path
            from deux_task_core.model import RobotModel,JOINT_NAMES
            from deux_task_runtime.trajectory import arrays
            robot=RobotModel(self.config['robot']);_,path=arrays(request.plan.trajectory,JOINT_NAMES)
            work=self.transfer_pool.submit(validate_conveyor_path,robot,path,lambda:goal.is_cancel_requested or self.estop)
            while not work.done():self.guard(goal,version);await pause(self,.01)
            clearance=float(work.result());request.plan.minimum_clearance=clearance
            if request.plan.stage=='ScanPose':self.scan_timing['minimum_conveyor_clearance_m']=clearance
        try:return await super().call(client,request,goal,version,timeout)
        except ValueError as error:
            if (client is self.executor_client and 'HAND_FEEDBACK_STALE' in str(error)
                and getattr(getattr(request,'plan',None),'stage','') in ('ScanPose','AlignBarcode')
                and not self.recovery_active and not goal.is_cancel_requested and not self.estop):
                raise TransientHandFeedbackError(f'{request.plan.stage} hand feedback delayed: {error}') from error
            if (client is self.planner and getattr(request,'stage','')=='AlignBarcode'
                and not goal.is_cancel_requested and not self.estop):
                raise RecoverableCycleError(f'AlignBarcode planning failed: {error}') from error
            raise

    async def place_after_scan(self,goal,version):
        if not self.get_parameter('parallel_reader_retreat').value:
            for name in stages(self.config):await self.stage(name,goal,version)
            return
        from deux_task_runtime.parallel_placement import build_parallel
        from deux_task_runtime.trajectory import validate_limits
        from deux_task_core.model import JOINT_NAMES,RobotModel
        from deux_task_core.ik_backends import backend_name
        from deux_task_msgs.msg import MotionPlan
        from deux_task_msgs.action import ExecuteMotion
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        from lookup_conveyor import validate_conveyor_path
        self.guard(goal,version);self.set_state('ParallelPlace')
        start=self.measured();work=self.transfer_pool.submit(build_parallel,self.config,start.copy(),lambda:goal.is_cancel_requested or self.estop)
        while not work.done():self.guard(goal,version);await pause(self,.01)
        data=work.result();robot=RobotModel(self.config['robot'])
        work=self.transfer_pool.submit(validate_conveyor_path,robot,data['path'],lambda:goal.is_cancel_requested or self.estop)
        while not work.done():self.guard(goal,version);await pause(self,.01)
        clearance=float(work.result());self.guard(goal,version)
        import numpy as np
        indices=np.r_[np.arange(7),np.arange(14,21)]
        if np.max(abs(self.measured()[indices]-start[indices]))>self.config['start_tolerance']:raise ValueError('arms moved during parallel planning')
        plan=MotionPlan(stage='PlaceApproach',plan_id=uuid4().hex,scene_version=version,target_id=self.target_id,
            model_id=self.config['model_id'],ik_backend=backend_name(self.config),synthetic=bool(self.config.get('synthetic',False)),minimum_clearance=clearance)
        plan.header.stamp=ros_time(self.now());plan.header.frame_id=self.config['base_frame'];plan.valid_until=ros_time(self.now()+self.config['max_plan_age'])
        plan.target_pose.orientation.w=1.;plan.trajectory.joint_names=list(JOINT_NAMES);plan.phase_end_times=data['phase_end_times'].tolist()
        plan.trajectory.points=[JointTrajectoryPoint(positions=q.tolist(),time_from_start=ros_time(float(t),Duration)) for t,q in zip(data['times'],data['path'])]
        validate_limits(plan.trajectory,JOINT_NAMES,robot)
        self.parallel_timing=dict(data['report'],minimum_conveyor_clearance_m=clearance)
        directory=Path(self.get_parameter('bt_output_directory').value);directory.mkdir(parents=True,exist_ok=True)
        try:(directory/f'parallel-place-{plan.plan_id}.json').write_text(json.dumps(dict(report=self.parallel_timing,times_s=data['times'].tolist(),positions_rad=data['path'].tolist(),phase_end_times=list(plan.phase_end_times)),indent=2))
        except (OSError,TypeError,ValueError) as error:self.get_logger().error(f'parallel plan log failed (motion continues): {error}')
        self.publish_status();result=await self.call(self.executor_client,ExecuteMotion.Goal(plan=plan),goal,version,float(data['times'][-1])+10.)
        if result.reason=='dry_run_only':raise ValueError('parallel placement checked only; no motion claimed')

    async def recover_cycle(self,goal,error):
        if not enabled(self.config) or not self.config['placement'].get('fixed_joint_motion',False):
            raise ValueError('automatic recovery requires configured fixed-bin placement')
        version=self.config_version;self.previous_clock=self.now()
        self.recovery_active=True;self.bt_goal=goal;self.close_session();self.perception_request=None
        self.repeat_last_error=str(error)
        record=dict(error=str(error),target_id=self.target_id,cycle=self.repeat_cycle,
                    repeat_session_id=self.repeat_session_id,stages=[],success=False)
        self.set_state('RecoverToBin');self.reason=str(error);self.publish_status();self.publish_repeat_status()
        try:
            if isinstance(error,TransientHandFeedbackError):
                await self.wait_feedback_health(goal)
                record['stages'].append('ScanPose')
                await self.stage('ScanPose',goal,version)
            await self.hold_perception(True)
            # Keep hand closed until the existing placement action reaches its
            # stationary release phase. Never open while returning over the belt.
            record['stages'].append('ParallelPlace' if self.get_parameter('parallel_reader_retreat').value else 'SequentialPlace')
            await self.place_after_scan(goal,version)
            record['success']=True;self.repeat_recovered+=1
            self.set_state('RecoveryComplete');self.reason='failed cycle released at configured bin and retreated; next cycle starts from ready'
            self.publish_status()
        except BaseException as recovery_error:
            record['recovery_error']=str(recovery_error);raise
        finally:
            self.recovery_active=False;self.bt_goal=None;self.close_session();self.perception_request=None
            try:await self.hold_perception(False)
            except Exception as release_error:self.get_logger().error(f'recovery perception release failed: {release_error}')
            try:
                directory=Path(self.get_parameter('bt_output_directory').value);directory.mkdir(parents=True,exist_ok=True)
                (directory/f'recovery-{uuid4().hex}.json').write_text(json.dumps(record,indent=2))
            except OSError as write_error:self.get_logger().error(f'recovery log failed: {write_error}')
            self.publish_repeat_status()

    async def execute(self,goal):
        self.repeat_session_id=uuid4().hex;self.repeat_cycle=0;self.repeat_completed=0
        self.repeat_failed=0;self.repeat_recovered=0;self.repeat_last_error=''
        self.repeat_target_id=goal.request.target_id
        if self.repeat_enabled and self.repeat_target_id:raise ValueError('repeat requires empty target_id')
        limit=int(self.get_parameter('repeat_max_cycles').value)
        if limit<0:raise ValueError('repeat_max_cycles must be nonnegative')
        try:
            if self.recover_before_next:
                self.recover_before_next=False
                await self.recover_cycle(goal,TransientHandFeedbackError('operator-requested recovery of previous failed scan cycle'))
            while True:
                if goal.is_cancel_requested or self.estop:raise InterruptedError('operator canceled / e-stop active')
                self.repeat_cycle+=1;self.publish_repeat_status()
                try:result=await self.execute_cycle(goal)
                except ScanObjectMissing as error:
                    self.repeat_failed+=1;self.repeat_last_error=str(error)
                    self.set_state('ViaEmptyReturnReady');self.reason=str(error);self.publish_status()
                    await self.hold_perception(False)
                    await self.child_call(self.empty_return_client,MoveToReady.Goal(),goal,self.config_version)
                    if not self.repeat_enabled:
                        return RunTask.Result(success=False,reason=str(error))
                    continue
                except RecoverableCycleError as error:
                    self.repeat_failed+=1;self.repeat_last_error=str(error);self.publish_repeat_status()
                    await self.recover_cycle(goal,error)
                    if not self.repeat_enabled:
                        self.state='Failed';self.reason=f'cycle failed; recovery complete: {error}';self.publish_status()
                        return RunTask.Result(success=False,reason=self.reason)
                    await pause(self,.25)
                    if not self.repeat_enabled:
                        self.state='Failed';self.reason=f'cycle failed; recovery complete: {error}';self.publish_status()
                        return RunTask.Result(success=False,reason=self.reason)
                    continue
                if not result.success:return result
                self.repeat_completed+=1;self.publish_repeat_status()
                if not self.repeat_enabled or (limit and self.repeat_completed>=limit):return result
                self.set_state('RepeatReady')
                self.reason=f'completed {self.repeat_completed} cycles; return to ready for next object'
                self.publish_status()
                await pause(self,.25)
                if not self.repeat_enabled:
                    self.state='Succeeded';self.reason=f'repeat disabled after {self.repeat_completed} completed cycles'
                    self.publish_status();return result
        except BaseException as error:
            self.repeat_enabled=False
            self.state='Canceled' if goal.is_cancel_requested or self.estop else 'Failed'
            self.reason=str(error);self.publish_status();raise
        finally:
            self.publish_repeat_status()

    async def execute_cycle(self,goal):
        version=self.config_version;self.previous_clock=self.now();self.target_id=''
        self.bt_goal=goal;self.bt_events=[];self.scan_timing={};self.parallel_timing={};self.scan_bbox_evidence={};run_id=uuid4().hex
        self.requested_target_id=goal.request.target_id;self.grasp_attempt=1;self.scan_attempt=0
        self.ik_calls=0;self.reason='';self.barcode=None;self.visual=None;self.label=None
        self.pre_grasp_surface_z=None;self.perception_request=None;self.last_lookup_result=None
        self.physical_pick_verified=False;self.lookup_committed=False;self.close_session()
        async def prepare():
            self.set_state('LookupPrepare')
            await self.hold_perception(False)
            await self.child_call(self.prepare_client,MoveToReady.Goal(),goal,version)
        async def grasp():
            self.set_state('LookupAcquire');request_id=uuid4().hex
            result=await self.child_call(self.lookup_client,LookupGrasp.Goal(request_id=request_id,
                target_id=self.requested_target_id,scene_version=version),goal,version,lookup=True)
            self.target_id=validate_handoff(result,request_id,self.requested_target_id)
            self.last_lookup_result=result
            if result.surface_height_valid:self.pre_grasp_surface_z=result.pre_grasp_surface_z
            await self.hold_perception(True)
            self.set_state('LookupHandoff')
            self.request_perception('barcode')
        async def present():
            # New replay already ends at scan; old servers retain their stage.
            if 'lift_scan_completed' in self.last_lookup_result.reason:
                import numpy as np
                if np.max(abs(self.measured()[14:21]-self.config['scan_right_joints']))>self.config['final_tolerance']:
                    raise ValueError('completed lookup scan endpoint differs from measured right arm')
                if 'left_scan_completed' in self.last_lookup_result.reason:
                    if self.dataset_reading_pose is None:raise ValueError('parallel left scan result requires matching pose configuration')
                    if np.max(abs(self.measured()[:7]-self.dataset_reading_pose['left_joints_rad']))>self.config['final_tolerance']:
                        raise ValueError('parallel left scan endpoint differs from measured left arm')
                self.set_state('ScanPose');return
            await self.stage('ScanPose',goal,version)
        async def verify_pick():
            if self.config.get('skip_visual_pick_verification',False):
                self.reason='visual pick verification skipped by existing configuration; motion is not grasp proof';self.publish_status();return
            if self.pre_grasp_surface_z is None:raise ValueError('pre-grasp visual height unavailable; cannot verify visual pick')
            self.set_state('VerifyVisualPick');samples=set()
            def visible_pick():
                if not self.visible(require_barcode=True) or not self.visual.surface_center_valid:return False
                obs=self.observation
                if obs is None or not obs.right_ee_valid or not current(obs.base_from_right_ee.header.stamp,self.now(),self.config['max_observation_age']):return False
                import numpy as np
                p=self.visual.surface_center;t=obs.base_from_right_ee.transform.translation
                if p.z<self.pre_grasp_surface_z+.04 or np.linalg.norm([p.x-t.x,p.y-t.y,p.z-t.z])>.20:return False
                samples.add(seconds(self.visual.header.stamp));return len(samples)>=3
            await self.await_condition(visible_pick,goal,version)
            # This is visual evidence only, not force/contact confirmation.
        async def align():
            self.scan_attempt+=1
            if self.dataset_reading_pose is not None:
                self.reason='fixed dataset left scan posture; visual barcode reacquisition not required'
                await self.stage('AlignBarcode',goal,version)
                return
            self.set_state('TrackBarcode')
            await self.await_condition(lambda:self.visible(True) and self.label is not None and
                current(self.label.header.stamp,self.now(),self.config['max_object_age']) and
                self.label.request_id==self.perception_request.request_id and self.label.target_id==self.target_id and
                self.label.sequence==self.visual.sequence and self.label.barcode_track_id==self.visual.barcode_track_id,
                goal,version)
            await self.stage('AlignBarcode',goal,version)
        async def read():
            if self.config.get('skip_barcode_completion',False):
                self.reason='barcode completion skipped by existing configuration; no read success claimed';self.publish_status();return
            self.set_state('WaitBarcode');self.session_id=uuid4().hex;self.scan_start=self.now();self.barcode=None
            self.session_pub.publish(ScanSession(header=Header(stamp=ros_time(self.scan_start)),session_id=self.session_id,
                track_id=self.target_id,expires_at=ros_time(self.scan_start+self.config['scan_timeout']),active=True))
            try:
                until=time.monotonic()+self.wall_timeout(self.config['scan_timeout'])
                while self.barcode is None and self.now()<self.scan_start+self.config['scan_timeout'] and time.monotonic()<until:
                    self.guard(goal,version);await pause(self)
                if self.barcode is None:raise Failure('barcode reader timed out')
            finally:self.close_session()
        async def barcode():
            try:await Sequence('Barcode',[Leaf('AlignBarcode',align),Leaf('ReadBarcode',read)]).run()
            except Failure as error:
                if self.scan_attempt>=3:raise RecoverableCycleError(f'barcode retries exhausted: {error}') from error
                raise
        async def place():
            self.close_session()
            await self.place_after_scan(goal,version)
        children=[Leaf('LookupPrepare',prepare),Leaf('LookupGraspAndLift',grasp),Leaf('ScanPose',present),
                  Leaf('VerifyVisualPick',verify_pick),Retry('BarcodeRetry',Leaf('BarcodeAttempt',barcode),3)]
        if enabled(self.config):children.append(Leaf('PlaceAndRetreat',place))
        self.tree=Sequence('LookupPickScanPlace',children)
        try:
            await self.tree.run();self.state='Succeeded'
            self.reason='lookup replay and configured scan/place motion flow completed; physical grasp/placement unverified'
            if self.config.get('skip_barcode_completion',False):self.reason+='; barcode completion skipped by configuration'
            return RunTask.Result(success=True,reason=self.reason,barcode=self.barcode.value if self.barcode else '')
        except BaseException as error:
            self.state='Canceled' if goal.is_cancel_requested or self.estop else 'Failed';self.reason=str(error);raise
        finally:
            self.close_session();self.perception_request=None;self.publish_status()
            try:await self.hold_perception(False)
            except Exception as error:self.get_logger().error(f'perception ownership release failed: {error}')
            self.bt_goal=None
            record=dict(run_id=run_id,repeat_session_id=self.repeat_session_id,cycle=self.repeat_cycle,state=self.state,reason=self.reason,target_id=self.target_id,
                physical_grasp_verified=False,physical_placement_verified=False,scan_bbox_evidence=self.scan_bbox_evidence,
                barcode_completed=bool(self.barcode),barcode_completion_skipped=bool(self.config.get('skip_barcode_completion',False)),
                visual_pick_check_skipped=bool(self.config.get('skip_visual_pick_verification',False)),scan_timing=self.scan_timing,parallel_timing=self.parallel_timing,events=self.bt_events)
            if self.last_lookup_result is not None:
                record.update(lookup_request_id=self.last_lookup_result.request_id,cell_id=self.last_lookup_result.cell_id,
                              grasp_mode=self.last_lookup_result.grasp_mode,path_completed=self.last_lookup_result.path_completed)
            try:
                directory=Path(self.get_parameter('bt_output_directory').value);directory.mkdir(parents=True,exist_ok=True)
                (directory/f'bt-{run_id}.json').write_text(json.dumps(record,indent=2))
            except OSError as error:self.get_logger().error(f'BT evidence write failed: {error}')

    def destroy_node(self):
        self.via_depth.close()
        self.transfer_pool.shutdown(wait=False,cancel_futures=True)
        return super().destroy_node()


def main():run(LookupTaskNode)


if __name__=='__main__':main()
