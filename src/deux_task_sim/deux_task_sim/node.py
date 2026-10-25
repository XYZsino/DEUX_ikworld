"""MuJoCo driver + explicitly simulated sensors on production ROS contracts."""
import time
import json
from copy import deepcopy
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from control_msgs.action import FollowJointTrajectory
from deux_hand_msgs.msg import HandCtlMessage
from deux_task_msgs.msg import ObjectDetectionArray,ObjectDetection,BarcodeTarget,PerceptionRequest,ScanSession,TaskStatus
from std_msgs.msg import String
from deux_task_core.model import JOINT_NAMES,ARM_NAMES,HAND_NAMES
from deux_task_runtime.common import ros_time,seconds,LATCHED,pause
from deux_task_runtime.trajectory import arrays,sample


def set_pose(out, pose):
    p,q=out.position,out.orientation
    p.x,p.y,p.z=map(float,pose[:3,3]);q.x,q.y,q.z,q.w=map(float,Rotation.from_matrix(pose[:3,:3].copy()).as_quat())


class MujocoNode(Node):
    def __init__(self,sim,config,output,*,reader_enabled=True,dropout=False,record=False):
        super().__init__('mujoco_driver_node')
        from transfer_sim_adapter import SimObservationSource,SimBarcodeSource
        self.group=ReentrantCallbackGroup();self.sim=sim;self.config=config;self.output=output
        self.record=record;self.video_frames=[];self.video_qpos=[];self.stage_qpos=[]
        if record:(output/'frames').mkdir(exist_ok=True)
        self.reader_enabled=reader_enabled;self.dropout=dropout
        self.source=SimObservationSource(sim,width=160,height=90,min_visible_pixels=4)
        self.reader=SimBarcodeSource(sim)
        self.request=None;self.session=None;self.stage='Idle';self.arm_goal=None
        self.arm_target=np.array([sim.data.qpos[int(sim.model.joint(n).qposadr[0])] for n in ARM_NAMES])
        self.hand_target={s:np.array([sim.data.qpos[int(sim.model.joint(n).qposadr[0])] for n in HAND_NAMES if n.startswith(s)]) for s in ('left','right')}
        self.arm_pub=self.create_publisher(JointState,'/arm/joint_states',qos_profile_sensor_data)
        self.hand_pub={s:self.create_publisher(JointState,f'/sim/hand/{s}/joint_states',qos_profile_sensor_data) for s in ('left','right')}
        self.clock_pub=self.create_publisher(Clock,'/clock',10)
        self.obj_pub=self.create_publisher(ObjectDetectionArray,'/task/object_detections',qos_profile_sensor_data)
        self.label_pub=self.create_publisher(BarcodeTarget,'/task/barcode_target',qos_profile_sensor_data)
        self.raw_pub=self.create_publisher(String,'/barcode/raw',10)
        self.tf=TransformBroadcaster(self)
        self.subs=[self.create_subscription(HandCtlMessage,f'/hand/{s}/cmd',lambda m,s=s:self.hand(s,m),10) for s in ('left','right')]
        self.subs += [self.create_subscription(PerceptionRequest,'/task/perception_request',lambda m:setattr(self,'request',m),LATCHED),
            self.create_subscription(ScanSession,'/task/scan_session',lambda m:setattr(self,'session',m),LATCHED),
            self.create_subscription(TaskStatus,'/task/status',self.status,LATCHED)]
        self.action=ActionServer(self,FollowJointTrajectory,'/arm/deux_arm_controller/follow_joint_trajectory',
            execute_callback=self.arm_execute,goal_callback=self.arm_accept,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
        self.busy=False;self.records=[];self.stage_events=[];self.reader_events=[];self.max_welds=0
        self.timer=self.create_timer(sim.nsub*sim.model.opt.timestep,self.step)
        self.frames=0

    def hand(self,side,msg):
        if msg.side != side or msg.mode != 7 or {m.id for m in msg.motors} != set(range(7)):return
        values=np.array([next(m.rad for m in msg.motors if m.id==i) for i in range(7)])
        if np.isfinite(values).all():self.hand_target[side]=values

    def arm_accept(self,req):
        try:
            arrays(req.trajectory,ARM_NAMES)
            if self.busy or seconds(req.trajectory.header.stamp)<self.sim.data.time:raise ValueError('busy/late')
        except ValueError:return GoalResponse.REJECT
        self.busy=True;return GoalResponse.ACCEPT

    async def arm_execute(self,goal):
        tr=goal.request.trajectory;t,q=arrays(tr,ARM_NAMES);start=seconds(tr.header.stamp)
        try:
            while self.sim.data.time < start+t[-1]+2.:
                if goal.is_cancel_requested:
                    self.arm_target=self.measured(ARM_NAMES);goal.canceled()
                    return FollowJointTrajectory.Result(error_code=-5,error_string='canceled')
                elapsed=self.sim.data.time-start
                if elapsed>=0:self.arm_target=sample(t,q,elapsed)
                if elapsed>=t[-1] and np.max(abs(self.measured(ARM_NAMES)-q[-1]))<.03:
                    goal.succeed();return FollowJointTrajectory.Result(error_code=0)
                await pause(self,.01)
            goal.abort();return FollowJointTrajectory.Result(error_code=-5,error_string='physical arm tracking timeout')
        finally:self.busy=False

    def measured(self,names):
        return np.array([self.sim.data.qpos[int(self.sim.model.joint(n).qposadr[0])] for n in names])

    def status(self,msg):
        if msg.state != self.stage:
            self.stage=msg.state;self.stage_events.append(dict(time=float(self.sim.data.time),stage=msg.state))
            self.get_logger().info(f'{self.sim.data.time:.3f}s {msg.state}')
            self.stage_qpos.append((msg.state,self.sim.data.qpos.copy()))

    def render_recording(self):
        """Render saved physical states after execution, avoiding render I/O latency in ROS callbacks."""
        import imageio.v3 as iio
        camera=mujoco.MjvCamera();camera.lookat[:]=[-.3,0.,.05]
        camera.distance=2.;camera.azimuth=130.;camera.elevation=-25.
        for index,(stage,qpos) in enumerate(self.stage_qpos):
            self.sim.data.qpos[:]=qpos;mujoco.mj_forward(self.sim.model,self.sim.data)
            self.sim.scene_renderer.update_scene(self.sim.data,camera=camera)
            iio.imwrite(str(self.output/f'{index+1:02}-{stage}.png'),self.sim.scene_renderer.render())
        for index,qpos in enumerate(self.video_qpos):
            self.sim.data.qpos[:]=qpos;mujoco.mj_forward(self.sim.model,self.sim.data)
            self.sim.renderer.update_scene(self.sim.data,camera=camera)
            iio.imwrite(str(self.output/'frames'/f'{index:06d}.jpg'),self.sim.renderer.render(),quality=85)
        if self.video_qpos:
            np.savez_compressed(self.output/'recorded-qpos.npz',qpos=np.array(self.video_qpos))

    def step(self):
        sim=self.sim
        wire=sim.home.copy();wire[:7]=self.arm_target[:7];wire[8:15]=self.arm_target[7:]
        sim.finger_targets={s:q.copy() for s,q in self.hand_target.items()}
        sim.model.actuator_forcerange[sim._hand_u['right']]=[-5.,5.]
        sim.command(wire);sim.advance();mujoco.mj_forward(sim.model,sim.data);stamp=ros_time(float(sim.data.time))
        self.clock_pub.publish(Clock(clock=stamp))
        for names,pub in [(ARM_NAMES,self.arm_pub)]+[(tuple(n for n in HAND_NAMES if n.startswith(s)),self.hand_pub[s]) for s in ('left','right')]:
            m=JointState();m.header.stamp=stamp;m.name=list(names);m.position=self.measured(names).tolist();pub.publish(m)
        from transfer_sim_adapter import transform
        base=np.linalg.inv(transform(sim.data,'base_link'))
        transforms=[]
        for s in ('left','right'):
            sid=sim._grasp_site[s];pose=np.eye(4);pose[:3,:3]=sim.data.site_xmat[sid].reshape(3,3);pose[:3,3]=sim.data.site_xpos[sid];pose=base@pose
            m=TransformStamped();m.header.stamp=stamp;m.header.frame_id='base_link';m.child_frame_id=s+'_grasp'
            p,q=m.transform.translation,m.transform.rotation;p.x,p.y,p.z=map(float,pose[:3,3]);q.x,q.y,q.z,q.w=map(float,Rotation.from_matrix(pose[:3,:3].copy()).as_quat());transforms.append(m)
        self.tf.sendTransform(transforms)
        if self.frames%6==0:
            frame=self.source.capture(cameras=('head','right_wrist','left_wrist'))
            out=ObjectDetectionArray();out.header.stamp=stamp;out.header.frame_id='base_link';out.sequence=frame.sequence
            out.source='sim_visibility_gated_truth_6d';out.model_id=self.config['model_id'];out.request_id=self.request.request_id if self.request else ''
            if not (self.dropout and self.stage=='Approach'):
                for d in frame.detections:
                    m=ObjectDetection(track_id=d.track_id,confidence=1.,full_orientation=True);set_pose(m.pose.pose,d.pose);out.detections.append(m)
            self.obj_pub.publish(out)
            self.label(frame,stamp)
        if self.reader_enabled and self.session and self.session.active and sim.data.time < seconds(self.session.expires_at):
            for read in self.reader.capture():
                if read.track_id==self.session.track_id:
                    self.raw_pub.publish(String(data=read.value));self.reader_events.append(dict(time=float(sim.data.time),value=read.value,target=read.track_id))
        else:self.reader.since.clear()
        self.max_welds=max(self.max_welds,int(np.count_nonzero(sim.data.eq_active)))
        obj=base@transform(sim.data,'bread_3')
        self.records.append(dict(time=float(sim.data.time),stage=self.stage,object_pose=obj.tolist(),joints=self.measured(JOINT_NAMES).tolist()))
        if self.record and self.frames%3==0:
            index=len(self.video_frames)
            self.video_qpos.append(sim.data.qpos.copy())
            self.video_frames.append(dict(frame=index,simulation_time_s=float(sim.data.time)))
        self.frames+=1
        if self.frames % 30 == 0:
            (self.output/'progress.json').write_text(json.dumps(dict(time=float(sim.data.time),stage=self.stage,
                object_position=obj[:3,3].tolist(),reader_events=len(self.reader_events))))
        if np.any(sim.data.warning.number):raise RuntimeError('MuJoCo physics warning')

    def label(self,frame,stamp):
        req=self.request
        if not req or req.mode!='barcode' or self.sim.data.time>=seconds(req.expires_at):return
        target=next((d for d in frame.detections if d.track_id==req.target_id),None)
        if target is None:return
        from reader_pick import LABELS
        from deux_conveyor import BREAD_MODEL
        options=[]
        for label in LABELS:
            normal=target.pose[:3,:3]@np.array(label['normal_mesh']);x=target.pose[:3,:3]@np.array(label['horizontal_mesh'],float)
            x-=normal*(x@normal);x/=np.linalg.norm(x)
            pose=np.eye(4);pose[:3,:3]=np.column_stack((x,np.cross(normal,x),normal))
            pose[:3,3]=target.pose[:3,:3]@(np.array(label['point_mesh_m'])-BREAD_MODEL['center_offset_m'])+target.pose[:3,3]
            # Emulated label detector chooses the face toward the current left reader.
            direction=frame.ee_left[:3,3]-pose[:3,3]
            options.append((direction@normal,pose))
        score,pose=max(options,key=lambda p:p[0])
        if score<=0:return
        m=BarcodeTarget(request_id=req.request_id,target_id=req.target_id,sequence=frame.sequence,
            orientation_valid=True,valid_fraction=1.,plane_rmse_m=0.)
        m.header.stamp=stamp;m.header.frame_id='base_link';set_pose(m.pose,pose);self.label_pub.publish(m)

    def destroy_node(self):
        self.source.close();self.sim.close();return super().destroy_node()
