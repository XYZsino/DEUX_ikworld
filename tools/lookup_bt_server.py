"""BT child actions for audited preparation and classified stored grasp replay."""
import json,time
from uuid import uuid4
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action.graph import get_action_names_and_types
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicHermiteSpline,BPoly
from std_msgs.msg import String
from std_srvs.srv import Trigger
from lookup_soft_stop import build_soft_stop
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from deux_task_msgs.action import LookupGrasp,MoveToReady
from deux_task_msgs.msg import VisualEvidenceArray,VisualEvidence
from deux_task_core.model import ARM_NAMES
from deux_task_runtime.common import ExclusiveAction,LATCHED,current,ros_time,seconds,pause,cancel_children
from classified_grasp_trial import ClassifiedTrial
from lookup_bt_support import prepare_plan,attention_plan
from lookup_empty_return import smooth_empty_return


class LookupBtServer(ClassifiedTrial):
    # Lookup-only posture tolerances; tracking and planning guards stay unchanged.
    ready_tolerance_rad=.05
    attention_tolerance_rad=.05
    left_ready_tolerance_rad=float(np.deg2rad(8.))
    action_type=LookupGrasp
    action_name='/task/lookup_grasp'

    def __init__(self):
        super().__init__()
        self.ready_action=ExclusiveAction(self,MoveToReady,'/task/lookup_prepare',self.prepare,self.admit_prepare,gate=self.action.gate)
        self.attention_action=ExclusiveAction(self,MoveToReady,'/task/lookup_attention',self.return_to_attention,self.admit_prepare,gate=self.action.gate)
        self.empty_return_action=ExclusiveAction(self,MoveToReady,'/task/lookup_empty_return',self.return_after_empty,self.admit_prepare,gate=self.action.gate)
        self.pool=ThreadPoolExecutor(max_workers=1)
        self.soft_stop_requested=False
        self.soft_stop_service=self.create_service(Trigger,'/task/lookup_soft_stop',self.request_soft_stop,callback_group=self.group)
        self.declare_parameter('ready_velocity_cap',0.45)
        self.declare_parameter('ready_acceleration_cap',0.9)
        for key in ('ready_velocity_cap','ready_acceleration_cap'):
            value=float(self.get_parameter(key).value)
            if not np.isfinite(value) or value<=0:raise ValueError(f'{key} must be positive and finite')
        self.declare_parameter('right_hand_ready_fraction',0.3)
        self.right_hand_ready_fraction=float(self.get_parameter('right_hand_ready_fraction').value)
        if not np.isfinite(self.right_hand_ready_fraction) or not 0.<=self.right_hand_ready_fraction<=1.:raise ValueError('right ready hand fraction must be within [0,1]')
        self.declare_parameter('dataset_reading_pose_file','')
        self.declare_parameter('scan_timing_profile','legacy')
        scan_profile=self.get_parameter('scan_timing_profile').value
        if scan_profile not in ('legacy','via_preserved_fast_scan'):raise ValueError('unknown scan timing profile')
        from deux_task_runtime.dataset_reading_pose import load_pose
        reading=load_pose(self.get_parameter('dataset_reading_pose_file').value)
        self.parallel_scan_left=reading['left_joints_rad'] if reading else None
        from lookup_height_policy import load_paths,same_heights
        from pathlib import Path
        baseline=json.loads((Path(__file__).resolve().parents[1]/'artifacts/dataset-barcode-pose/replay-height-baseline.json').read_text())
        self.height_baseline=baseline['heights']
        same,current=same_heights(self.config,self.height_baseline)
        self.get_logger().info('Installation heights unchanged: reuse offline paths, NO trajectory revalidation' if same else 'Installation heights changed: trajectory validation required')
        self.lift_scan_future=self.pool.submit(load_paths,self.library,self.config,
            lambda n,ok,bad:self.get_logger().info(f'changed-height validation {n}: accepted={ok}, excluded={bad}'),left_target=self.parallel_scan_left,profile=scan_profile)
        self.get_logger().info(f'Lookup scan timing profile: {scan_profile}')
        self.lift_scan_paths=None
        self.completed_targets=set();self.current_id='';self.requested_id='';self.active_goal=None;self.ever_committed=False;self.seen_requests=set()
        self.surface_candidates=None;self.surface=None;self.selected_surface=None
        self.create_subscription(VisualEvidenceArray,'/task/visual_candidates',lambda m:setattr(self,'surface_candidates',m),qos_profile_sensor_data)
        self.create_subscription(VisualEvidence,'/task/visual_evidence',lambda m:setattr(self,'surface',m),qos_profile_sensor_data)
        self.bt_state=self.create_publisher(String,'/task/lookup_state',LATCHED)
        self.create_timer(.05,self.feedback_tick,callback_group=self.group)

    def replay_path(self,key):
        if self.lift_scan_paths is None:raise ValueError('lift-scan compilation not ready')
        return self.lift_scan_paths[key]

    def validate_replay(self,goal,curve,start_at,close=False):
        actual=super().validate_replay(goal,curve,start_at,close)
        left=getattr(curve,'left_curve',None)
        if left is not None and self.now()>=start_at:
            expected=left(min(self.now()-start_at,float(curve.x[-1])))
            if np.max(abs(actual[:7]-expected))>self.config['tracking_tolerance']:
                self.warn_execution('parallel left scan tracking error')
        return actual

    def prepare_guard(self,goal=None):
        if self.estop or (self.hand is not None and getattr(self.hand,'estop',False)):raise InterruptedError('e-stop active')
        if goal is not None and goal.is_cancel_requested:raise InterruptedError('operator canceled')
        self.hardware_gate();self.check_model_version()
        if time.monotonic()-self.external<1.:raise ValueError('competing direct arm command')
        return self.measured()

    def ready_error(self):
        # Report actual right-arm error; admission uses ready_tolerance_rad.
        return float(np.max(abs(self.measured()[7:]-self.library.ready[14:21])))

    def gate(self,goal=None):
        actual=self.prepare_guard(goal)
        if self.locked_selection is None and np.max(abs(actual[:7]-self.library.ready[:7]))>self.left_ready_tolerance_rad:
            raise ValueError('left installation home outside configured final tolerance')
        return actual

    def phase_details(self):
        details=super().phase_details()
        details.update(left_ready_source='installation.yaml home_joints[:7]',
                       ready_error_scope='right arm only; separate lookup left home tolerance',
                       right_ready_tolerance_rad=self.ready_tolerance_rad,
                       attention_tolerance_rad=self.attention_tolerance_rad,
                       left_ready_tolerance_rad=self.left_ready_tolerance_rad)
        try:details['left_home_error_rad']=float(np.max(abs(self.measured()[:7]-self.library.ready[:7])))
        except ValueError:pass
        return details

    def admit_prepare(self,request):
        if not self.enabled:raise ValueError('lookup BT preview only')
        self.prepare_guard()
        if not self.client.server_is_ready():raise ValueError('arm controller unavailable')

    def admit(self,request):
        if not self.enabled:raise ValueError('lookup BT preview only')
        if request is not None:
            if not request.request_id or request.request_id in self.seen_requests:raise ValueError('fresh request_id required')
            if request.scene_version!=self.config_version:raise ValueError('lookup scene version mismatch')
        self.gate()
        if self.lift_scan_paths is None:raise ValueError('run lookup preparation to finish lift-scan validation')
        if self.ready_error()>self.ready_tolerance_rad:raise ValueError('audited lookup ready pose required')
        if not self.client.server_is_ready():raise ValueError('arm controller unavailable')
        self.hand.guard()
        if any(name=='/task/grid_grasp' for name,_ in get_action_names_and_types(self)):
            raise ValueError('standalone grasp server conflicts with full BT')

    def on_target_wait_timeout(self):
        self.warn_execution('no eligible target yet; continue waiting without trajectory dispatch')

    def tracked_observation(self):
        row=super().tracked_observation();self.current_id=row[0]
        if row[0] in self.completed_targets:raise ValueError('completed target; waiting for a new object')
        if self.requested_id and row[0]!=self.requested_id:raise ValueError('waiting for requested lookup target')
        # Capture independent visible surface BEFORE the path is committed.
        if self.locked_selection is None:
            candidates=[]
            if self.surface_candidates is not None:candidates.extend(self.surface_candidates.candidates)
            if self.surface is not None:candidates.append(self.surface)
            for candidate in candidates:
                if (candidate.target_id==row[0] and candidate.surface_center_valid and candidate.bag_visible
                    and candidate.header.frame_id=='base_link' and current(candidate.header.stamp,self.now(),self.config['max_object_age'])):
                    self.selected_surface=candidate;break
        return row

    def feedback_tick(self):
        active=self.action.busy or self.ready_action.busy or self.attention_action.busy or self.empty_return_action.busy
        phase='returning_to_attention' if self.attention_action.busy else 'empty_return' if self.empty_return_action.busy else 'preparing' if self.ready_action.busy else 'acquiring'
        elapsed=-1.;selected=self.locked_selection
        if selected is not None:
            self.ever_committed=True;elapsed=self.now()-selected['start_at_s']
            phase=('scheduled' if elapsed<0 else 'approach' if elapsed<2. else
                   'descend_close' if elapsed<3. else 'lift' if elapsed<4. else 'right_via_to_scan')
        self.bt_state.publish(String(data=json.dumps(dict(active=active,phase=phase,
            committed=self.ever_committed,stamp_s=self.now()))))
        if self.active_goal is not None and self.active_goal.is_active:
            self.active_goal.publish_feedback(LookupGrasp.Feedback(state=phase,committed=self.ever_committed,
                target_id=(selected or {}).get('target_id',self.current_id),cell_id=(selected or {}).get('cell_id',''),elapsed_s=elapsed))

    async def execute(self,goal):
        req=goal.request;self.seen_requests.add(req.request_id);self.requested_id=req.target_id
        self.soft_stop_requested=False
        self.active_goal=goal;self.locked_selection=None;self.execution_warnings=[];self.ever_committed=False;self.selected_surface=None;self.last_replay_report={};self.used=False
        try:
            basic=await super().execute(goal);report=self.last_replay_report
            report['request_id']=req.request_id
            if basic.success and report.get('left_scan_parallel'):
                try:
                    actual=self.prepare_guard(goal)
                    if np.max(abs(actual[:7]-self.parallel_scan_left))>self.config['final_tolerance']:
                        self.warn_execution('parallel left scan final joint error')
                except InterruptedError:raise
                except Exception as error:self.warn_execution('parallel left final feedback: '+str(error))
            try:(self.out/f'lookup-{uuid4().hex}.json').write_text(json.dumps(report,indent=2))
            except OSError as error:self.get_logger().error(f'lookup evidence write failed: {error}')
            result=LookupGrasp.Result(success=basic.success,reason=basic.reason,request_id=req.request_id,
                target_id=report.get('target_id',''),cell_id=report.get('cell_id',''),grasp_mode=report.get('grasp_mode',''),
                path_completed=basic.success,physical_grasp_verified=False)
            if report.get('replay_stop_confirmed'):result.reason+='; replay_stop_confirmed'
            if report.get('soft_stop'):result.reason+='; soft_stop_completed'
            if basic.success and report.get('scan_completed'):result.reason+='; lift_scan_completed'
            if basic.success and report.get('left_scan_parallel'):result.reason+='; left_scan_completed'
            if report.get('cell_id'):
                result.acquired_at=ros_time(report['acquired_at_s']);result.contact_at=ros_time(report['contact_at_s'])
                key=report['cell_id'];p,curve,info=self.replay_path(key);end=float(curve.x[-1])
                for field,t,stamp in [(result.contact_grasp,3.,report['contact_at_s']),
                                      (result.stopped_grasp,end,report['start_at_s']+end)]:
                    q=self.library.ready.copy();q[14:21]=curve(t)
                    if getattr(curve,'left_curve',None) is not None:q[:7]=curve.left_curve(t)
                    pose=self.robot.ee_fk('right',q);field.header.frame_id='base_link';field.header.stamp=ros_time(stamp)
                    field.pose.position.x,field.pose.position.y,field.pose.position.z=map(float,pose[:3,3])
                    quat=Rotation.from_matrix(pose[:3,:3]).as_quat()
                    field.pose.orientation.x,field.pose.orientation.y,field.pose.orientation.z,field.pose.orientation.w=map(float,quat)
            if self.selected_surface is not None and self.selected_surface.target_id==result.target_id:
                result.surface_height_valid=True;result.pre_grasp_surface_z=self.selected_surface.surface_center.z
            if result.success:self.completed_targets.add(result.target_id)
            return result
        finally:self.active_goal=None;self.requested_id=''

    def request_soft_stop(self,request,response):
        if self.estop or self.active_goal is None or self.active_goal.is_cancel_requested or self.locked_selection is None:
            response.success=False;response.message='no active committed replay eligible for soft stop'
        else:
            self.soft_stop_requested=True
            response.success=True;response.message='controlled deceleration requested'
        return response

    def prepare_soft_stops(self,plan,info,goal):
        # Audit candidate stops while the original grasp/lift is running, not at
        # the moving inspection deadline. Each starts on the selected path;
        # independently timed joints can leave its synchronized geometry.
        via=info.get('waypoint_at_s',6.)
        self.soft_stop_candidates=[(float(t),self.pool.submit(build_soft_stop,plan,float(t),self.robot,
            self.library.ready.copy(),lambda:goal.is_cancel_requested or self.estop))
            for t in np.arange(via+.55,min(via+1.4,float(plan['times_s'][-1])-.21),.1)]

    async def stop_for_missing(self,goal,plan,start_at):
        pending=handle=None;completed=False
        try:
            self.prepare_guard(goal)
            options=[(t,f.result()) for t,f in self.soft_stop_candidates
                     if start_at+t>self.now()+.10 and f.done() and f.exception() is None]
            if not options:raise ValueError('no prevalidated soft stop available before scan end')
            elapsed,data=options[0];start=start_at+elapsed
            tr=JointTrajectory(joint_names=list(ARM_NAMES));tr.header.stamp=ros_time(start)
            for t,q,v in zip(data['times_s'],data['positions_rad'],data['velocities_rad_s']):
                tr.points.append(JointTrajectoryPoint(positions=q,velocities=v,time_from_start=ros_time(t,Duration)))
            pending=self.client.send_goal_async(self.replay_goal(tr))
            while not pending.done():
                self.prepare_guard(goal)
                if self.now()>=start:raise TimeoutError('soft-stop acceptance deadline')
                await pause(self,.005)
            handle=pending.result()
            if not handle.accepted:raise ValueError('soft-stop controller rejected replacement')
            if self.now()>=start:raise TimeoutError('soft-stop accepted too late')
            future=handle.get_result_async();until=time.monotonic()+data['duration_s']+3.
            curve=CubicHermiteSpline(data['times_s'],data['positions_rad'],data['velocities_rad_s'])
            while not future.done():
                actual=self.prepare_guard(goal)
                if self.now()>=start and np.max(abs(actual-curve(np.clip(self.now()-start,0.,data['duration_s']))))>self.config['tracking_tolerance']:
                    raise ValueError('soft-stop tracking error')
                if time.monotonic()>until:raise TimeoutError('soft-stop completion deadline')
                await pause(self,.005)
            response=future.result()
            if response.status!=4 or response.result.error_code:raise ValueError('soft-stop controller failed')
            await self.settled_arms(goal)
            completed=True
            self.get_logger().info(f"bag missing soft stop completed: {data['duration_s']:.2f}s deceleration; "
                f"profile={data.get('profile','common_path')}; ee_travel={data.get('ee_travel',{})}")
            return data
        finally:
            self.soft_stop_requested=False
            if not completed:
                if pending is not None and handle is None:
                    def late(f):
                        if f.result().accepted:f.result().cancel_goal_async()
                    pending.add_done_callback(late)
                await cancel_children(self,[handle])

    async def return_to_attention(self,goal):
        return await self.prepare(goal,stow=True)

    async def settled_arms(self,goal):
        last=self.prepare_guard(goal);last_time=settled_since=time.monotonic()
        deadline=last_time+5.
        while time.monotonic()-settled_since<.3:
            if time.monotonic()>deadline:raise ValueError('arms not settled for attention return')
            await pause(self,.05)
            measured=self.prepare_guard(goal);now=time.monotonic()
            if np.max(abs(measured-last))/max(now-last_time,.001)>.03:settled_since=now
            last=measured;last_time=now
        return last

    async def return_after_empty(self,goal):
        return await self.prepare(goal,smooth=True)

    async def prepare(self,goal,stow=False,smooth=False):
        strict=stow or smooth
        self.locked_selection=None;self.ever_committed=False
        pending=handle=None;stop=False
        try:
            from lookup_height_policy import ROOT,same_heights
            import yaml
            current_height_config=yaml.safe_load((ROOT/'src/deux_task_bringup/config/installation.yaml').read_text())
            if not same_heights(current_height_config,self.height_baseline)[0] or (ROOT/'.setup/camera-calibration-lift-invalid.json').exists():
                raise ValueError('Installed lift/conveyor heights changed; regenerate paths before another cycle')
            while not stow and not self.lift_scan_future.done():
                if goal.is_cancel_requested or self.estop:raise InterruptedError('operator canceled / e-stop active')
                await pause(self,.02)
            await pause(self,.05)
            if not stow and self.lift_scan_paths is None:
                from lookup_lift_scan import restrict_selection
                self.lift_scan_paths,rejected=self.lift_scan_future.result()
                restrict_selection(self.library,self.lift_scan_paths)
                self.get_logger().info(f'Offline replay paths ready: {len(self.lift_scan_paths)} paths, {len(rejected)} excluded')
            # Also required after a moving-via presence check cancels the grasp action.
            actual=await self.settled_arms(goal)
            work=self.pool.submit(attention_plan if stow else smooth_empty_return if smooth else prepare_plan,self.config,self.library.ready.copy(),actual.copy(),lambda:goal.is_cancel_requested,
                                  float(self.get_parameter('ready_velocity_cap').value),float(self.get_parameter('ready_acceleration_cap').value))
            while not work.done():self.prepare_guard(goal);await pause(self,.02)
            data=work.result();actual_now=self.prepare_guard(goal)
            if np.max(abs(actual_now-actual))>.03:raise ValueError('arms changed during ready planning')
            try:
                (self.out/f'{"attention" if stow else "prepare"}-{uuid4().hex}.json').write_text(json.dumps(dict(
                    times_s=data['times'].tolist(),positions_rad=data['path'].tolist(),
                    minimum_clearance_m=data['minimum_clearance'],no_motion=data['no_motion'],smooth_empty_return=smooth,
                    acceleration_cap_rad_s2=data.get('acceleration_cap'),jerk_cap_rad_s3=data.get('jerk_cap')),indent=2))
            except OSError as error:self.get_logger().error(f'prepare evidence write failed: {error}')
            if data['no_motion']:return MoveToReady.Result(success=True,reason='audited arms already at attention' if stow else 'audited arms already ready')
            indices=np.r_[np.arange(7),np.arange(14,21)]
            tr=JointTrajectory(joint_names=list(ARM_NAMES));start=self.now()+.75;tr.header.stamp=ros_time(start)
            for t,q in zip(data['times'],data['path']):
                tr.points.append(JointTrajectoryPoint(positions=q[indices].tolist(),velocities=[0.]*14,accelerations=[0.]*14 if smooth else [],time_from_start=ros_time(t,Duration)))
            curve=(BPoly.from_derivatives(data['times'],[[q[indices],np.zeros(14),np.zeros(14)] for q in data['path']]) if smooth else
                   CubicHermiteSpline(data['times'],data['path'][:,indices],np.zeros((len(data['times']),14))))
            pending=self.client.send_goal_async(self.replay_goal(tr))
            def monitor():
                if strict:return self.prepare_guard(goal)
                try:return self.prepare_guard(goal)
                except InterruptedError:raise
                except Exception as error:self.warn_execution(str(error));return None
            acceptance_deadline=time.monotonic()+5.
            while not pending.done():
                if strict and time.monotonic()>acceptance_deadline:raise TimeoutError('attention controller response timeout')
                monitor();await pause(self,.01)
            handle=pending.result()
            if not handle.accepted:raise ValueError('ready controller request rejected')
            if strict and self.now()>=start:raise ValueError('attention trajectory accepted after its scheduled start')
            future=handle.get_result_async()
            finish_deadline=time.monotonic()+float(data['times'][-1])+10.
            while not future.done():
                if strict and time.monotonic()>finish_deadline:raise TimeoutError('attention controller completion timeout')
                measured=monitor()
                if measured is not None and np.max(abs(measured-curve(np.clip(self.now()-start,0.,data['times'][-1]))))>self.config['tracking_tolerance']:
                    if strict:raise ValueError('strict return tracking error')
                    self.warn_execution('lookup ready tracking error')
                await pause(self,.02)
            response=future.result()
            if response.status!=4 or response.result.error_code:raise ValueError('lookup ready controller failed')
            final=self.measured()
            if stow:
                final=await self.settled_arms(goal)
                if np.max(abs(final))>self.attention_tolerance_rad:raise ValueError(f'attention final error {np.max(abs(final)):.4f} rad exceeds {self.attention_tolerance_rad:.4f} rad')
                return MoveToReady.Result(success=True,reason='both arms returned to attention via reversed simultaneous ready route; fingers unchanged')
            if self.ready_error()>self.ready_tolerance_rad or np.max(abs(final[:7]-self.library.ready[:7]))>self.left_ready_tolerance_rad:
                raise ValueError('ready final pose outside configured arm tolerances')
            return MoveToReady.Result(success=True,reason='installation left home / stored right ready restored; conveyor-only geometry; hand commands unchanged')
        except BaseException as error:
            stop=strict or isinstance(error,(InterruptedError,KeyboardInterrupt)) or goal.is_cancel_requested or self.estop
            raise
        finally:
            if stop:
                if pending is not None and handle is None:
                    def late(f):
                        if f.result().accepted:f.result().cancel_goal_async()
                    pending.add_done_callback(late)
                await cancel_children(self,[handle])


def main():
    rclpy.init();node=LookupBtServer()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.pool.shutdown(wait=False,cancel_futures=True);node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
