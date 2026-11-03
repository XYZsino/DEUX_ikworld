"""Acquisition-matched RGB-D/arm feedback for the moving-via presence gate."""
from collections import deque
from pathlib import Path
import numpy as np


def box_depth(depth,xyxy,ee_depth,tolerance=.15,scale=1.,excluded=None,polygon=None):
    """Depth outside projected hands in inner bbox/optional instance polygon."""
    if not np.isfinite(ee_depth) or ee_depth<=0:raise ValueError('EE behind camera / invalid depth')
    x0,y0,x1,y1=map(float,xyxy);dx=(x1-x0)*.05;dy=(y1-y0)*.05
    x0=max(0,int(np.ceil(x0+dx)));x1=min(depth.shape[1],int(np.floor(x1-dx)))
    y0=max(0,int(np.ceil(y0+dy)));y1=min(depth.shape[0],int(np.floor(y1-dy)))
    patch=depth[y0:y1,x0:x1];usable=np.ones(patch.shape,bool)
    if excluded is not None:usable &= ~excluded[y0:y1,x0:x1]
    if polygon:
        import cv2
        shape=np.zeros(depth.shape,np.uint8);cv2.fillPoly(shape,[np.asarray(polygon).reshape(-1,2).astype(np.int32)],1)
        usable &= shape[y0:y1,x0:x1].astype(bool)
    remaining=int(usable.sum())
    if remaining<100 or remaining/max(1,patch.size)<.15:raise ValueError('too little bag area outside hand mask')
    valid=patch[usable & np.isfinite(patch)&(patch>0)]
    fraction=valid.size/max(1,remaining)
    if valid.size<30 or fraction<.3:raise ValueError('insufficient valid bbox depth')
    measured=float(np.median(valid))*scale;delta=measured-ee_depth
    return dict(depth_passed=abs(delta)<=tolerance,box_depth_m=measured,ee_depth_m=float(ee_depth),
                depth_delta_m=float(delta),depth_tolerance_m=float(tolerance),valid_depth_fraction=float(fraction),remaining_pixels=remaining,excluded_pixels=int(patch.size-remaining))


class ViaDepthGate:
    def __init__(self,node):
        from deux_task_perception.camera_input import CameraInput
        from deux_task_core.model import RobotModel
        from tf2_ros import Buffer,TransformListener
        from sensor_msgs.msg import JointState
        from rclpy.qos import qos_profile_sensor_data
        self.node=node;self.frames=deque(maxlen=32);self.joints=deque(maxlen=200);self.hands=deque(maxlen=200)
        self.robot=RobotModel(node.config['robot'])
        from via_hand_mask import HandSilhouettes
        from via_frame_archive import FrameArchive
        self.silhouettes=HandSilhouettes(node.config['robot']['urdf_path'])
        node.declare_parameter('via_frame_directory',str(Path(__file__).resolve().parents[1]/'.setup/lookup-bt/via-frames'))
        self.archive=FrameArchive(node.get_parameter('via_frame_directory').value,error_callback=lambda error:node.get_logger().error('capture storage error (independent of detection): '+error))
        self.tf=Buffer();self.listener=TransformListener(self.tf,node)
        self.camera=CameraInput(node,lambda msg:self.frames.append(msg))
        self.hand_subscription=node.create_subscription(JointState,'/task/hand_state',lambda msg:self.hands.append(msg),qos_profile_sensor_data)
        self.subscription=node.create_subscription(JointState,'/arm/joint_states',lambda msg:self.joints.append(msg),qos_profile_sensor_data)

    def filter(self,message,matches,tolerance):
        from rclpy.time import Time
        from deux_task_core.model import ARM_NAMES
        from deux_task_perception.core import stamp_ns,depth_view,joint_values
        from deux_task_perception.perception_core import transform_matrix
        stamp=stamp_ns(message.header)
        if (Path(__file__).resolve().parents[1]/'.setup/camera-calibration-lift-invalid.json').exists():
            raise ValueError('camera calibration invalidated')
        frames=[f for f in self.frames if stamp_ns(f.header)==stamp and f.header.frame_id==message.header.frame_id]
        if not frames:raise ValueError('matching registered depth frame unavailable')
        frame=frames[-1]
        from deux_task_perception.core import normalize_rgb
        from deux_task_core.model import HAND_NAMES
        depth,scale=depth_view(frame.depth);excluded=None;accepted=[];evidence=[]
        metadata=dict(stamp_ns=stamp,request_id=message.request_id,frame_id=frame.header.frame_id,
            min_confidence=(getattr(self.node,'scan_bbox_context',None) or {}).get('min_confidence',.5),
            depth_tolerance_m=float(tolerance),roi=(getattr(self.node,'scan_bbox_context',None) or {}).get('roi'),
            detector_id=message.detector_id,projection=list(frame.rgb_info.p),depth_unit='m',
            depth_source_encoding=frame.depth.encoding,hand_padding_px=8,
            boxes=[dict(class_name=b.class_name,confidence=float(b.confidence),xyxy=list(b.xyxy),polygon_xy=list(b.polygon_xy)) for b in message.boxes])
        try:
            samples=[j for j in self.joints if 0<=stamp-stamp_ns(j.header)<=100_000_000]
            hand_samples=[j for j in self.hands if 0<=stamp-stamp_ns(j.header)<=100_000_000]
            if not samples or not hand_samples:raise ValueError('image-time arm/hand feedback unavailable')
            sample=max(samples,key=lambda j:stamp_ns(j.header));arms=joint_values(sample,ARM_NAMES)
            hand=max(hand_samples,key=lambda j:stamp_ns(j.header));fingers=joint_values(hand,HAND_NAMES)
            q=np.asarray(self.node.config['home_joints'],float).copy();q[:7]=arms[:7];q[14:21]=arms[7:]
            q[7:14]=fingers[:7];q[21:28]=fingers[7:]
            tf=self.tf.lookup_transform(frame.header.frame_id,self.node.config['base_frame'],
                Time.from_msg(frame.header.stamp,clock_type=self.node.get_clock().clock_type))
            t=tf.transform.translation;r=tf.transform.rotation
            camera_from_base=transform_matrix([t.x,t.y,t.z],[r.x,r.y,r.z,r.w])
            ee=camera_from_base@self.robot.ee_fk('right',q);z=float(ee[2,3])
            excluded=self.silhouettes.project(self.robot,q,camera_from_base,frame.rgb_info.p,depth.shape)
            metadata.update(joints_rad=q.tolist(),arm_stamp_ns=stamp_ns(sample.header),hand_stamp_ns=stamp_ns(hand.header),
                camera_from_base=camera_from_base.tolist(),ee_depth_m=z,excluded_pixels=int(excluded.sum()))
            for match in matches:
                try:
                    result=box_depth(depth,match['xyxy'],z,tolerance,scale,excluded,match.get('polygon_xy'))
                    evidence.append(dict(class_name=match['class_name'],xyxy=match['xyxy'],**result))
                    if result['depth_passed']:accepted.append(dict(match,**result))
                except ValueError as error:evidence.append(dict(class_name=match['class_name'],depth_passed=False,reason=str(error)))
        except Exception as error:evidence.append(dict(depth_passed=False,reason=str(error)))
        metadata.update(accepted=accepted,depth_evidence=evidence)
        try:
            path=self.archive.save(message.request_id,stamp,normalize_rgb(frame.rgb),(depth.astype(np.float32)*scale),excluded,metadata)
            evidence.append(dict(archive_path=path,archive_queued=True))
        except Exception as error:
            # A storage problem must never erase a successful depth result.
            evidence.append(dict(archive_error=str(error),archive_queued=False))
        return accepted,evidence

    def recording_status(self):return self.archive.status()

    def close(self):self.archive.close()
