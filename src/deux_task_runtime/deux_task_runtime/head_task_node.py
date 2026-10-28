"""Image-first PickScan BT. ICP is required only while acquiring the grasp.

Visual evidence after grasp is explicitly a visible target near the hand, not
force/contact proof and not a fabricated full object pose.
"""
from copy import deepcopy
from uuid import uuid4
import time
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from deux_task_msgs.action import RunTask
from deux_task_msgs.msg import DetectionArray, VisualEvidence, VisualEvidenceArray, ScanSession
from .task_node import TaskNode
from .behavior_tree import Leaf, Sequence, Retry, Failure
from .common import current, seconds, ros_time, pose_matrix, pause, run


class HeadTaskNode(TaskNode):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        if self.config.get('perception_policy')!='head_rgbd':
            raise ValueError('HeadTaskNode requires head_rgbd installation policy')
        self.workflow="head_rgbd_pick_scan"
        from deux_task_core.placement import enabled
        if enabled(self.config):
            self.workflow='head_rgbd_pick_scan_place'
        self.pre_grasp_surface_z=None
        self.grasp_objects=None
        self.tracks=None;self.visual=None;self.candidates=None;self.track_history={}
        self.track_sub=self.create_subscription(DetectionArray,'/task/detections',self.receive_tracks,qos_profile_sensor_data)
        self.visual_sub=self.create_subscription(VisualEvidence,'/task/visual_evidence',lambda m:setattr(self,'visual',m),qos_profile_sensor_data)

        self.candidate_sub=self.create_subscription(VisualEvidenceArray,'/task/visual_candidates',
            self.receive_candidates,qos_profile_sensor_data)

    def receive_candidates(self,msg):
        self.candidates=msg
        # Perception runs while the robot prepares; only the later BT stages
        # authorize approach. This avoids spending the interception window on
        # Kalman warm-up after the hand is already ready.
        if self.state!='Prepare' or not self.config.get('moving_source',False) or self.target_id:return
        req=self.perception_request
        if req is None or msg.request_id!=req.request_id or req.mode!='detect':return
        if msg.header.frame_id!=self.config['base_frame'] or not current(msg.header.stamp,self.now(),self.config['max_object_age']):return
        if self.config.get('target_selection_policy')=='rgb_roi_3d_bbox':
            from .roi_selection import select_roi_target
            if select_roi_target(self,self.requested_target_id):
                self.prepare_target_last_seen=seconds(msg.header.stamp)
                self.request_perception('locate')
            return
        from deux_task_core.conveyor import inside
        cfg=self.config['conveyor_tracking']
        stable={key for key,values in self.track_history.items() if len(values)>=3}
        direction=np.asarray(cfg['flow_direction'])
        entry=np.where(direction>=0,cfg['grasp_region']['min'],cfg['grasp_region']['max'])
        upstream_limit=float(np.dot(entry,direction))-cfg['boundary_margin']
        options=[c for c in msg.candidates if c.target_id in stable and c.bag_visible and c.surface_center_valid
                 and inside([c.surface_center.x,c.surface_center.y,c.surface_center.z],cfg['acquisition_region'])
                 and np.dot([c.surface_center.x,c.surface_center.y,c.surface_center.z],direction)<upstream_limit
                 and (not self.requested_target_id or c.target_id==self.requested_target_id)]
        if not options:return
        chosen=min(options,key=lambda c:np.dot([c.surface_center.x,c.surface_center.y,c.surface_center.z],cfg['flow_direction']))
        self.target_id=chosen.target_id
        self.prepare_target_last_seen=seconds(msg.header.stamp)
        self.request_perception('locate')

    def validate(self, req):
        self.robot_ready()
        if not self.planner.server_is_ready() or not self.executor_client.server_is_ready():
            raise ValueError('planner/executor unavailable')

    def receive_tracks(self,msg):
        stamp=seconds(msg.header.stamp)
        if self.tracks is not None:
            previous=seconds(self.tracks.header.stamp)
            if stamp==previous:return
            if stamp<previous or msg.detector_id!=self.tracks.detector_id:self.track_history.clear()
        threshold=0. if self.config.get('target_selection_policy')=='rgb_roi_3d_bbox' else .5
        seen={b.track_id for b in msg.boxes if b.class_name=='bread_bag' and b.confidence>=threshold and b.track_id}
        self.track_history={key:[s for s in values if stamp-s<=.6] for key,values in self.track_history.items() if key in seen}
        for key in seen:self.track_history[key]=(self.track_history.get(key,[])+[stamp])[-3:]
        self.tracks=msg
        if self.state=='Prepare' and self.config.get('moving_source',False) and self.target_id:
            if self.target_id in seen:self.prepare_target_last_seen=stamp
            elif stamp-getattr(self,'prepare_target_last_seen',stamp)>.35:
                # Image IDs may change under the preparing arm. Reselect only
                # BEFORE any approach; never transfer an active grasp to a new ID.
                self.target_id='';self.request_perception('detect')

    def planning_observation(self, stage):
        obs=deepcopy(self.observation)
        if not self.config.get('moving_source',False) and stage in ('Approach','Descend','Close'):
            if self.grasp_objects is None:raise ValueError('no acquired grasp pose')
            obs.objects=deepcopy(self.grasp_objects)
            obs.objects_valid=True
        if self.config.get('moving_source',False) and stage in ('Approach','TrackGrasp'):
            cfg=self.config['conveyor_tracking']
            if cfg.get('occluded_grasp_enabled',False):
                try:
                    self.target(self.target_id,obs=obs)
                    self.grasp_objects=deepcopy(obs.objects)
                except ValueError:
                    if self.grasp_objects is None:raise ValueError('no previous conveyor observation')
                    obs.objects=deepcopy(self.grasp_objects);obs.objects_valid=True
                    self.target(self.target_id,obs=obs,max_age=cfg.get('approach_prediction_max_age',self.config['max_object_age']) if stage=='Approach' else cfg['occlusion_prediction_max_age'])
        return obs

    def rendezvous_ready(self):
        from deux_task_core.conveyor import conveyor_velocity,rendezvous_ready,validate_occlusion_prediction
        from deux_task_msgs.msg import MotionPlan
        cfg=self.config['conveyor_tracking']
        obs=self.planning_observation('TrackGrasp')
        obj=self.target(self.target_id,obs=obs,max_age=cfg['occlusion_prediction_max_age'])
        if not obj.pose_filtered or not obj.velocity_valid:return False
        if not obs.right_ee_valid or not current(obs.base_from_right_ee.header.stamp,self.now(),self.config['max_observation_age']):return False
        v=obj.velocity;velocity=conveyor_velocity([v.linear.x,v.linear.y,v.linear.z,v.angular.z],self.config)
        age=self.now()-seconds(obs.objects.header.stamp)
        if age>self.config['max_object_age']:
            plan=MotionPlan(stage='TrackGrasp',target_pose=obj.pose.pose,target_acquired_at=obs.objects.header.stamp)
            plan.target_velocity.linear.x,plan.target_velocity.linear.y,plan.target_velocity.linear.z=map(float,velocity[:3])
            plan.target_velocity.angular.z=float(velocity[3])
            validate_occlusion_prediction(obj,plan,age,self.now(),self.config)
        from geometry_msgs.msg import Pose
        tf=obs.base_from_right_ee.transform
        ee=Pose();ee.position.x=tf.translation.x;ee.position.y=tf.translation.y;ee.position.z=tf.translation.z;ee.orientation=tf.rotation
        return rendezvous_ready(pose_matrix(obj.pose.pose),velocity,age,pose_matrix(ee),self.config)

    def guard(self, goal, version):
        super().guard(goal, version)
        if self.config.get('moving_source',False) and self.state in ('Approach','WaitIntercept'):
            try:
                obj=self.target(self.target_id)
                if obj.pose_filtered and obj.velocity_valid:self.grasp_objects=deepcopy(self.observation.objects)
            except ValueError:pass
        if not self.config.get('moving_source',False) and self.state in ('Approach','Descend','Close') and self.grasp_objects is not None:
            if not current(self.grasp_objects.header.stamp,self.now(),self.config['grasp_pose_max_age']):
                raise ValueError('original grasp pose expired')
            if not self.visible():
                obj=next(d for d in self.grasp_objects.detections if d.track_id==self.target_id)
                ee=self.observation.base_from_right_ee
                if not self.observation.right_ee_valid or not current(ee.header.stamp,self.now(),self.config['max_observation_age']):
                    raise ValueError('measured hand pose unavailable during occlusion')
                p=obj.pose.pose.position;t=ee.transform.translation
                if np.linalg.norm([p.x-t.x,p.y-t.y,p.z-t.z])>.25:
                    raise ValueError('target lost before hand entered grasp region')

    def set_state(self,state):
        self.state=state;self.reason="";self.publish_status()

    def visible(self, require_barcode=False):
        m=self.visual;r=self.perception_request
        if (m is None or r is None or m.request_id!=r.request_id or m.target_id!=self.target_id
                or m.header.frame_id!=self.config['base_frame'] or not m.bag_visible
                or not current(m.header.stamp,self.now(),self.config['max_object_age'])):
            return False
        if require_barcode and (not m.barcode_visible or not m.barcode_track_id):return False
        return True

    async def await_condition(self, predicate, goal, version, timeout=8.):
        wall=time.monotonic();start=self.now()
        while self.now()-start < timeout and time.monotonic()-wall < self.wall_timeout(timeout):
            self.guard(goal,version)
            if predicate():return
            await pause(self)
        detail=f' — {self.reason}' if self.reason else ''
        raise Failure(f'{self.state}: fresh visual evidence timeout{detail}')

    def select_ready(self, observations, requested_id=''):
        """Admit a same-frame ROI/depth box, or retain the explicit legacy policy."""
        if self.config.get('target_selection_policy')=='rgb_roi_3d_bbox':
            from .roi_selection import select_roi_target
            return select_roi_target(self,requested_id)
        def blocked(reason):
            self.reason=reason
            return False
        m=self.tracks
        if m is None:return blocked('검출·추적 데이터 수신 대기')
        age=self.now()-seconds(m.header.stamp)
        if not current(m.header.stamp,self.now(),self.config['max_object_age']):
            return blocked(f'검출·추적 데이터 시각 불충족: 경과 {age:.2f}초, 허용 {self.config["max_object_age"]:.2f}초')
        detected=[b for b in m.boxes if b.class_name=='bread_bag']
        if not detected:return blocked('빵 봉투 미검출: 현재 영상의 bread_bag 0개')
        if requested_id:
            detected=[b for b in detected if b.track_id==requested_id]
            if not detected:return blocked(f'지정 대상 ID 미검출: {requested_id}')
        confident=[b for b in detected if b.confidence>=.5]
        if not confident:
            return blocked(f'봉투 검출 신뢰도 부족: 최대 {max(b.confidence for b in detected):.2f} < 0.50')
        bags=[b for b in confident if b.track_id]
        if not bags:return blocked(f'추적 ID 발급 대기: 신뢰도 조건을 만족한 봉투 {len(confident)}개')
        if self.config.get('moving_source',False):
            from deux_task_core.conveyor import inside
            candidates=self.candidates
            if candidates is None:return blocked(f'봉투 {len(bags)}개 검출, 깊이 기반 3D 후보 수신 대기')
            if candidates.request_id!=self.perception_request.request_id:
                return blocked('현재 대상 선택 요청에 대응하는 3D 후보 대기')
            if not current(candidates.header.stamp,self.now(),self.config['max_object_age']):
                age=self.now()-seconds(candidates.header.stamp)
                return blocked(f'3D 후보 데이터 시각 불충족: 경과 {age:.2f}초, 허용 {self.config["max_object_age"]:.2f}초')
            region=self.config['conveyor_tracking']['acquisition_region']
            by_id={c.target_id:c for c in candidates.candidates}
            counts=dict(missing=0,invisible=0,depth=0,outside=0);allowed=set();outside=[]
            for b in bags:
                c=by_id.get(b.track_id)
                if c is None:counts['missing']+=1;continue
                if not c.bag_visible:counts['invisible']+=1;continue
                xyz=np.array([c.surface_center.x,c.surface_center.y,c.surface_center.z])
                if not c.surface_center_valid or not np.isfinite(xyz).all():counts['depth']+=1;continue
                if not inside(xyz,region):
                    counts['outside']+=1;outside.append((b.track_id,xyz));continue
                allowed.add(b.track_id)
            bags=[b for b in bags if b.track_id in allowed]
            if not bags:
                labels={'missing':'같은 ID의 3D 후보 없음','invisible':'봉투 가시성 부족',
                        'depth':'유효한 깊이 좌표 없음','outside':'선택 영역 밖'}
                detail=', '.join(f'{labels[k]} {v}개' for k,v in counts.items() if v)
                if outside:
                    identity,xyz=outside[0];xyz=xyz*100
                    bounds=' / '.join(f'{axis} {lo*100:.1f}~{hi*100:.1f}' for axis,lo,hi in zip('XYZ',region['min'],region['max']))
                    detail+=f'; 예: {identity} XYZ=[{xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}]cm, 허용 {bounds}cm'
                return blocked('선택 가능한 3D 후보 없음: '+detail)
        for b in bags:observations.setdefault(b.track_id,set()).add(seconds(m.header.stamp))
        stable=[b for b in bags if len(observations[b.track_id])>=3]
        if not stable:
            best=max(bags,key=lambda b:len(observations[b.track_id]))
            return blocked(f'추적 관측 횟수 부족: {best.track_id} {len(observations[best.track_id])}/3회 (동일 영상은 중복 집계하지 않음)')
        if self.config.get('moving_source',False):
            direction=np.asarray(self.config['conveyor_tracking']['flow_direction'])
            by_id={c.target_id:c.surface_center for c in self.candidates.candidates}
            chosen=min(stable,key=lambda b:np.dot([by_id[b.track_id].x,by_id[b.track_id].y,by_id[b.track_id].z],direction))
        else:
            chosen=min(stable,key=lambda b:((b.xyxy[0]+b.xyxy[2])/2-480.)**2+((b.xyxy[1]+b.xyxy[3])/2-240.)**2)
        self.target_id=chosen.track_id;self.reason=''
        return True

    def estimate_ready(self, samples):
        """Expose the current blocking condition through TaskStatus.reason."""
        def blocked(reason):
            self.reason=reason
            return False
        obs=self.observation
        if obs is None:return blocked('ROS 관측 데이터 수신 대기')
        if obs.objects.request_id!=self.perception_request.request_id:
            return blocked('선택 대상 ICP 요청에 대응하는 결과 대기')
        try:
            target=self.target(self.target_id)
            if self.config.get('moving_source',False):
                from deux_task_core.conveyor import grasp_entry_prediction,conveyor_velocity
                cfg=self.config['conveyor_tracking']
                if not target.pose_filtered:return blocked('선택 대상의 칼만 필터 포즈 대기')
                if not target.velocity_valid:return blocked('선택 대상의 유효한 이동 속도 추정 대기')
                v=target.velocity;velocity=np.array([v.linear.x,v.linear.y,v.linear.z,v.angular.z])
                if self.config.get('target_selection_policy')=='rgb_roi_3d_bbox':
                    # Selection already admitted one RGB/depth box. Keep the
                    # minimum usable ICP velocity; actual plan enforces reach,
                    # forecast timing, collision and joint constraints.
                    conveyor_velocity(velocity,self.config)
                    self.reason=''
                    return True
                if not samples or samples[-1][0]!=obs.objects.sequence:
                    samples.append((obs.objects.sequence,velocity[:3]));del samples[:-3]
                count=cfg['min_motion_observations']
                if target.filter_observations<count:
                    return blocked(f'칼만 관측 횟수 부족: {target.filter_observations}/{count}회')
                if len(samples)<3:return blocked(f'속도 안정성 확인 중: {len(samples)}/3회')
                spread=max(np.linalg.norm(a[1]-b[1]) for a in samples for b in samples)
                limit=cfg['velocity_stability_m_s']
                if spread>limit:return blocked(f'속도 불안정: 변화 {spread*100:.2f}cm/s > 허용 {limit*100:.2f}cm/s')
                # Use the same checked velocity prior as the motion planner.
                velocity=conveyor_velocity(velocity,self.config)
                prediction=grasp_entry_prediction(self.config,pose_matrix(target.pose.pose),velocity,
                    self.now()-seconds(obs.objects.header.stamp))
                failure=prediction['failure']
                if failure=='acquisition':return blocked('현재 물체 위치가 대상 선택 영역 밖입니다')
                if failure:
                    key='arrival' if failure=='arrival' else 'completion'
                    label='접근 도착' if failure=='arrival' else '파지 완료'
                    xyz=prediction[key][:3,3]*100
                    horizon=prediction[key+'_s']
                    return blocked(f'{label} 예측 위치가 파지 영역 밖: 관측 후 {horizon:.2f}초, '
                        f'XYZ=[{xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}]cm '
                        f'(남은 동작 예상 {prediction["remaining_s"]:.2f}초)')
            self.reason=''
            return True
        except ValueError as exc:
            reasons={'objects unavailable':'유효한 ICP 물체 결과 대기',
                     'object stale/future':'ICP 결과 시각이 유효하지 않거나 오래됨',
                     'required object pose unavailable':'선택 ID의 ICP 포즈 없음',
                     'object frame/model origin mismatch':'ICP 좌표계 또는 모델 불일치',
                     'observed motion disagrees with conveyor command':'추정 속도가 설정된 컨베이어 속도와 불일치'}
            return blocked(reasons.get(str(exc),f'포즈·예측 조건 불충족: {exc}'))

    async def execute(self, goal):
        version=self.config_version;self.previous_clock=self.now()
        self.grasp_attempt=self.scan_attempt=self.ik_calls=0
        self.reason='';self.barcode=None;self.target_id='';self.visual=None;self.grasp_objects=None
        self.grasp_candidate_id=''
        self.requested_target_id=goal.request.target_id
        self.request_perception('detect')

        async def select():
            self.set_state('TrackAndSelect')
            if self.target_id and self.perception_request.mode=='locate':
                try:
                    self.target(self.target_id)
                    return
                except ValueError:
                    self.target_id='';self.request_perception('detect')
            observations={key:set(values) for key,values in self.track_history.items()}
            def candidate():return self.select_ready(observations,goal.request.target_id)
            await self.await_condition(candidate,goal,version)

        async def estimate():
            self.set_state('EstimatePose')
            if self.perception_request.mode!='locate':self.request_perception('locate')
            samples=[]
            def ready():return self.estimate_ready(samples)
            await self.await_condition(ready,goal,version,15.)
            self.grasp_objects=deepcopy(self.observation.objects)
            if not self.config.get('moving_source',False):self.request_perception('detect')
            def surface_ready():
                valid=self.visible() and self.visual.surface_center_valid
                self.reason='' if valid else '선택 대상의 최신 영상·깊이 표면 확인 대기'
                return valid
            await self.await_condition(surface_ready,goal,version)
            self.pre_grasp_surface_z=self.visual.surface_center.z

        async def grasp():
            self.grasp_attempt=1
            if self.config.get('moving_source',False):
                # Intercept using a short forecast; then sample a NEW observation
                # for the speed-matched tracking/descent/closure trajectory.
                await self.stage('Approach',goal,version)
                cfg=self.config['conveyor_tracking']
                from deux_task_core.time_candidates import settings as time_settings
                if (cfg.get('candidate_search',{}).get('enabled',False)
                        and time_settings(self.config).get('enabled',False)):
                    pass  # Selected absolute handoff replaces the fixed rendezvous wait.
                elif cfg.get('intercept_wait_s',0.):
                    self.set_state('WaitIntercept')
                    await self.await_condition(self.rendezvous_ready,goal,version,cfg['intercept_wait_timeout_s'])
                else:
                    arrived=self.now()
                    await self.await_condition(lambda:self.observation is not None and self.observation.objects_valid and seconds(self.observation.objects.header.stamp)>arrived,goal,version)
                await self.stage('TrackGrasp',goal,version)
            else:
                for stage in ('Approach','Descend','Close'):await self.stage(stage,goal,version)
            # Closing motion completion immediately triggers Lift. No grasp
            # success/contact/visual confirmation is required before lifting.
            # Stop submitting ICP jobs before moving a held, deformable bag.
            self.request_perception('barcode')
            await self.stage('Lift',goal,version)
            await self.stage('ScanPose',goal,version)

        async def reacquire():
            self.set_state('ReacquireHeldTarget');samples={}
            def unique_near_hand():
                m=self.candidates;obs=self.observation
                if (m is None or m.request_id!=self.perception_request.request_id or
                    m.header.frame_id!=self.config['base_frame'] or
                    not current(m.header.stamp,self.now(),self.config['max_object_age']) or
                    obs is None or not obs.right_ee_valid or
                    not current(obs.base_from_right_ee.header.stamp,self.now(),self.config['max_observation_age'])):
                    samples.clear();return False
                from .visual_selection import held_candidate
                t=obs.base_from_right_ee.transform.translation
                selected=held_candidate(m.candidates,[t.x,t.y,t.z],self.pre_grasp_surface_z)
                if selected is None:samples.clear();return False
                for key in list(samples):
                    if key!=selected.target_id:del samples[key]
                observed=samples.setdefault(selected.target_id,set())
                observed.add(seconds(m.header.stamp))
                if len(observed)<3:return False
                previous=self.target_id;self.target_id=selected.target_id
                self.get_logger().info(f'Visual hand association: {previous} -> {self.target_id}; not persistent identity proof')
                self.request_perception('barcode');return True
            await self.await_condition(unique_near_hand,goal,version)

        async def verify_visual():
            self.set_state('VerifyVisualPick');samples=set()
            def valid():
                if not self.visible(require_barcode=True) or not self.visual.surface_center_valid:return False
                obs=self.observation
                if obs is None or not obs.right_ee_valid:return False
                if not current(obs.base_from_right_ee.header.stamp,self.now(),self.config['max_observation_age']):return False
                p=self.visual.surface_center;t=obs.base_from_right_ee.transform.translation
                if self.pre_grasp_surface_z is None or p.z < self.pre_grasp_surface_z+.04:return False
                if np.linalg.norm(np.array([p.x-t.x,p.y-t.y,p.z-t.z]))>.20:return False
                samples.add(seconds(self.visual.header.stamp));return len(samples)>=3
            await self.await_condition(valid,goal,version)

        async def align():
            self.scan_attempt+=1;self.set_state('TrackBarcode')
            await self.await_condition(lambda:self.visible(True) and self.label is not None and
                current(self.label.header.stamp,self.now(),self.config['max_object_age']) and
                self.label.barcode_track_id==self.visual.barcode_track_id,goal,version)
            await self.stage('AlignBarcode',goal,version)

        async def read():
            self.set_state('WaitBarcode');self.session_id=uuid4().hex
            self.scan_start=self.now();self.barcode=None
            from std_msgs.msg import Header
            self.session_pub.publish(ScanSession(header=Header(stamp=ros_time(self.scan_start)),session_id=self.session_id,
                track_id=self.target_id,expires_at=ros_time(self.scan_start+self.config['scan_timeout']),active=True))
            try:
                # The aligned reader can occlude the printed code from the head
                # camera. Keep fresh selected-bag evidence; the bounded session
                # reader response supplies the independent read confirmation.
                # AlignBarcode still requires fresh tracked barcode geometry.
                await self.await_condition(lambda:self.visible() and self.barcode is not None,
                    goal,version,self.config['scan_timeout'])
            finally:self.close_session()

        async def place():
            # Reader clearance is completed BEFORE the right arm travels to the bin.
            self.close_session()
            self.request_perception('detect')
            for name in ('ClearReader','PlaceApproach','Place'):
                await self.stage(name,goal,version)
            await self.stage('Release',goal,version)
            await self.stage('PlaceRetreat',goal,version)
            await self.stage('Retreat',goal,version)

        children=[
            Leaf('Prepare',lambda:self.prepare_once(goal,version)),
            Leaf('TrackAndSelect',select),
            Leaf('EstimatePose',estimate),Leaf('GraspAndPresent',grasp),
            Leaf('ReacquireHeldTarget',reacquire),Leaf('VerifyVisualPick',verify_visual),
            Retry('ScanRetry',Sequence('TrackedScan',[Leaf('TrackBarcode',align),Leaf('WaitBarcode',read)]),3)]
        from deux_task_core.placement import enabled
        if enabled(self.config):children.append(Leaf('PlaceAndRetreat',place))
        self.tree=Sequence('HeadRgbdPickScanPlace' if enabled(self.config) else 'HeadRgbdPickScan',children)
        try:
            await self.tree.run()
            self.state='Succeeded';self.reason='bag visible near hand; tracked barcode and reader response observed'
            if enabled(self.config):self.reason='barcode read; release and retreat motions completed; placement confirmation disabled by operator'
            return RunTask.Result(success=True,reason=self.reason,barcode=self.barcode.value)
        except Exception as exc:
            self.state='Canceled' if goal.is_cancel_requested else 'Failed';self.reason=str(exc);raise
        finally:
            self.close_session();self.request_perception('detect');self.publish_status()


def main(args=None):run(HeadTaskNode,args)
