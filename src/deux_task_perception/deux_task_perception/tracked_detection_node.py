"""Continuous image tracking; selected-target ICP before grasp, visual surfaces after.

This node consumes registered RGB-D only. Object IDs come from the tracker and
label association uses current image containment, never a simulated bag pose.
"""
from copy import deepcopy, copy
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from deux_task_msgs.msg import (Observation, BoundingBox, DetectionFrame, DetectionArray,
                               PerceptionRequest, BarcodeTarget, VisualEvidence, VisualEvidenceArray)
from .pipeline_node import PipelineNode, run, LATEST_QOS
from .core import normalize_rgb, normalize_depth, validate_bundle, stamp_ns
from .yolo_backend import YoloTracker
from .icp_node import matrix
from .barcode_geometry import surface
from .perception_core import masked_depth
from .visible_geometry import surface_bounds, workspace_roi
from deux_task_core.configuration import load_configuration


class TrackedDetectionNode(PipelineNode):
    def __init__(self, detector=None, *, node_name="tracked_detection_node", extra_defaults=None, icp_transport=True, **kwargs):
        defaults = dict(input_topic='/task/observation',config_file='',
            weights='',device='cpu',confidence=.25,width=640,height=480,sync_slop_s=.02,
            base_frame='base_link',track_max_age_s=.75,bag_class='bread_bag',barcode_class='barcode')
        defaults.update(extra_defaults or {})
        super().__init__(node_name, defaults, **kwargs)
        self.selection_config = load_configuration(self.cfg['config_file'])[0] if self.cfg['config_file'] else {}
        self.detector=detector or YoloTracker(self.cfg['weights'],self.cfg['device'],self.cfg['confidence'],self.cfg['track_max_age_s'])
        self.request=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.detections=self.create_publisher(DetectionArray,'/task/detections',qos_profile_sensor_data)
        self.icp=(self.create_publisher(DetectionFrame,'/task/detection_frames',LATEST_QOS)
                  if icp_transport else None)
        self.labels=self.create_publisher(BarcodeTarget,'/task/barcode_target',qos_profile_sensor_data)
        self.candidates=self.create_publisher(VisualEvidenceArray,'/task/visual_candidates',qos_profile_sensor_data)
        self.evidence=self.create_publisher(VisualEvidence,'/task/visual_evidence',qos_profile_sensor_data)
        self.subscription=self.create_subscription(Observation,self.cfg['input_topic'],self.enqueue,LATEST_QOS)
        self.requests=self.create_subscription(PerceptionRequest,'/task/perception_request',self.set_request,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.start_worker(self.detect)

    def now(self):
        epoch=self.epoch
        now=super().now()
        if epoch!=self.epoch and hasattr(self,'request'):self.request=None
        return now

    def set_request(self, msg):
        now=self.now()
        if (msg.mode not in ('detect','locate','barcode','idle') or not msg.request_id
                or not 0 < stamp_ns(msg.header) <= now < self.expiry(msg)
                or self.expiry(msg)-now > 120e9):
            self.reject('invalid visual perception lease');return
        if self.request and stamp_ns(msg.header)<=stamp_ns(self.request.header):return
        self.request=deepcopy(msg)

    @staticmethod
    def expiry(msg): return msg.expires_at.sec*10**9+msg.expires_at.nanosec

    def detect(self, obs):
        if obs.header!=obs.rgb.header:raise ValueError('RGB acquisition mismatch')
        validate_bundle(obs.rgb,obs.depth,obs.rgb_info,obs.depth_info,
            self.cfg['width'],self.cfg['height'],int(self.cfg['sync_slop_s']*1e9))
        out=DetectionFrame(header=deepcopy(obs.header),sequence=obs.sequence,
            detector_id=self.detector.identity,depth=obs.depth,depth_info=obs.depth_info)
        for b in self.detector.track(normalize_rgb(obs.rgb),stamp_ns(obs.header)*1e-9):
            if b['class_name'] not in (self.cfg['bag_class'],self.cfg['barcode_class']):continue
            out.boxes.append(BoundingBox(class_id=b['class_id'],class_name=b['class_name'],
                confidence=b['confidence'],track_id=b['track_id'],xyxy=b['xyxy'],polygon_xy=b.get('polygon_xy',[])))
        return out

    def _poll(self):
        now=self.now()
        if self.worker is None:return
        done=self.worker.poll()
        if done is None:return
        try:
            epoch,frame=done.result()
            if epoch!=self.epoch or not self.fresh(frame.header,now):raise ValueError('expired image tracks')
            req=self.request
            if req and (now>=self.expiry(req) or stamp_ns(req.header)>stamp_ns(frame.header)):req=None
            frame.request_id=req.request_id if req else ''
            detections=DetectionArray(header=frame.header,sequence=frame.sequence,
                request_id=frame.request_id,detector_id=frame.detector_id,boxes=frame.boxes)
            if self.selection_config.get('target_selection_policy')=='rgb_roi_3d_bbox':
                try:
                    when=Time.from_msg(frame.header.stamp,clock_type=self.get_clock().clock_type)
                    bc=matrix(self.tf.lookup_transform(self.cfg['base_frame'],frame.header.frame_id,when))
                    detections.selection_roi_xyxy=workspace_roi(
                        self.selection_config['conveyor_tracking']['acquisition_region'],
                        bc,frame.depth_info.p,frame.depth.width,frame.depth.height,
                        self.selection_config.get('selection_roi_vertical_fraction')).tolist()
                    detections.selection_roi_valid=True
                except Exception as exc:self.reject(f'RGB selection ROI unavailable: {exc}')
            self.detections.publish(detections)
            if req:
                bags=[b for b in frame.boxes if b.class_name==self.cfg['bag_class']]
                if req.target_id:
                    targets=[b for b in bags if b.track_id==req.target_id]
                    if req.mode=='locate':
                        selected=copy(frame);selected.boxes=targets;self.submit_icp(selected)
                    self.publish_visual(frame,req,bags,targets)
                if req.mode in ('detect','barcode'):
                    candidates=[]
                    for bag in bags:
                        candidate_req=deepcopy(req);candidate_req.target_id=bag.track_id
                        candidates.append(self.publish_visual(frame,candidate_req,bags,[bag],candidate_only=True))
                    header=deepcopy(frame.header);header.frame_id=self.cfg['base_frame']
                    self.candidates.publish(VisualEvidenceArray(header=header,sequence=frame.sequence,
                        request_id=req.request_id,candidates=candidates))
            self.accepted+=1
        except Exception as exc:self.reject(str(exc))

    def submit_icp(self, frame):
        self.icp.publish(frame)

    def publish_visual(self,frame,req,bags,targets,candidate_only=False):
        out=VisualEvidence(header=deepcopy(frame.header),sequence=frame.sequence,
            request_id=req.request_id,target_id=req.target_id,bag_visible=len(targets)==1)
        out.header.frame_id=self.cfg['base_frame']
        if len(targets)!=1:self.evidence.publish(out);return
        selected=targets[0]
        candidates=[]
        for b in frame.boxes:
            if b.class_name!=self.cfg['barcode_class']:continue
            x,y=(np.asarray(b.xyxy[:2])+b.xyxy[2:])/2
            containers=[bag for bag in bags if bag.xyxy[0]<=x<=bag.xyxy[2] and bag.xyxy[1]<=y<=bag.xyxy[3]]
            if len(containers)==1 and containers[0].track_id==req.target_id:candidates.append(b)
        out.barcode_visible=len(candidates)==1
        if out.barcode_visible:out.barcode_track_id=candidates[0].track_id
        try:
            when=Time.from_msg(frame.header.stamp,clock_type=self.get_clock().clock_type)
            bc=matrix(self.tf.lookup_transform(self.cfg['base_frame'],frame.header.frame_id,when))
            depth,_=normalize_depth(frame.depth)
            x0,y0,x1,y1=np.asarray(selected.xyxy,int)
            x0,x1=np.clip([x0,x1],0,depth.shape[1]);y0,y1=np.clip([y0,y1],0,depth.shape[0])
            bag_depth=masked_depth(depth,selected.polygon_xy) if selected.polygon_xy else depth
            patch=bag_depth[y0:y1,x0:x1];ys,xs=np.where(np.isfinite(patch)&(patch>0)&(patch<3.))
            if len(xs)>=30:
                z=patch[ys,xs];keep=np.abs(z-np.median(z))<.04
                p=np.asarray(frame.depth_info.p).reshape(3,4)
                points=np.column_stack(((xs+x0-p[0,2])*z/p[0,0],(ys+y0-p[1,2])*z/p[1,1],z))
                center=(bc@np.r_[np.median(points[keep],axis=0),1.])[:3] if keep.sum()>=30 else None
                if center is not None:
                    out.surface_center_valid=True
                    out.surface_center.x,out.surface_center.y,out.surface_center.z=map(float,center)
            # Same geometry is used for the GUI box and selection evidence.
            if selected.polygon_xy:
                try:
                    lo,hi=surface_bounds(depth,frame.depth_info.p,selected.polygon_xy,bc)
                    out.bbox3d_min.x,out.bbox3d_min.y,out.bbox3d_min.z=map(float,lo)
                    out.bbox3d_max.x,out.bbox3d_max.y,out.bbox3d_max.z=map(float,hi)
                    out.bbox3d_valid=True
                except ValueError:pass
            if not candidate_only and req.mode=='barcode' and out.barcode_visible:
                pose,rmse,fraction=surface(depth,frame.depth_info.p,candidates[0].xyxy,3.)
                pose=bc@pose
                label=BarcodeTarget(header=out.header,sequence=frame.sequence,request_id=req.request_id,
                    target_id=req.target_id,barcode_track_id=out.barcode_track_id,
                    orientation_valid=True,plane_rmse_m=rmse,valid_fraction=fraction)
                label.pose.position.x,label.pose.position.y,label.pose.position.z=map(float,pose[:3,3])
                q=Rotation.from_matrix(pose[:3,:3]).as_quat()
                label.pose.orientation.x,label.pose.orientation.y,label.pose.orientation.z,label.pose.orientation.w=map(float,q)
                self.labels.publish(label)
        except Exception as exc:self.reject(f'visual geometry unavailable: {exc}')
        if not candidate_only:self.evidence.publish(out)
        return out


def main(args=None):run(TrackedDetectionNode,args)
