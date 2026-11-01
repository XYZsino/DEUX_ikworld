"""Continuous image tracking; selected-target ICP before grasp, visual surfaces after.

This node consumes registered RGB-D only. Object IDs come from the tracker and
label association uses current image containment, never a simulated bag pose.
"""
from copy import deepcopy, copy
import json
from queue import SimpleQueue
from std_msgs.msg import String
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
from .barcode_geometry import surface, center_position
from .perception_core import masked_depth
from .visible_geometry import surface_bounds, workspace_roi
from deux_task_core.configuration import load_configuration


class TrackedDetectionNode(PipelineNode):
    def __init__(self, detector=None, *, node_name="tracked_detection_node", extra_defaults=None, icp_transport=True, **kwargs):
        defaults = dict(input_topic='/task/observation',camera_input='observation',config_file='',
            barcode_position_only=False,weights='',device='cpu',confidence=.25,barcode_confidence=-1.,width=640,height=480,sync_slop_s=.02,
            base_frame='base_link',track_max_age_s=.75,bag_class='bread_bag',barcode_class='barcode',
            input_bag_classes='bread_bag',input_barcode_class='barcode',ignore_tracking_ids=False,
            single_target_tracking=False,tracking_roi_file='',tracking_require_roi_entry=True)
        defaults.update(extra_defaults or {})
        super().__init__(node_name, defaults, **kwargs)
        self.selection_config = load_configuration(self.cfg['config_file'])[0] if self.cfg['config_file'] else {}
        barcode_confidence = self.cfg['barcode_confidence']
        if barcode_confidence == -1.:
            barcode_confidence = self.cfg['confidence']
        if not np.isfinite(barcode_confidence) or not 0 <= barcode_confidence <= 1:
            raise ValueError('barcode_confidence must be -1 or within [0, 1]')
        self.barcode_confidence = barcode_confidence
        self.detector=detector or (None if self.cfg['single_target_tracking'] else YoloTracker(self.cfg['weights'],self.cfg['device'],
            min(self.cfg['confidence'],barcode_confidence),self.cfg['track_max_age_s']))
        if self.cfg['single_target_tracking']:
            if self.cfg['ignore_tracking_ids']:raise ValueError('single tracking requires real track IDs')
            from .single_target_backend import SingleTargetDetector
            from .tracking_roi import default_path
            self.detector=detector or SingleTargetDetector(self.cfg['weights'],self.cfg['device'],self.cfg['confidence'],
                [s.strip() for s in self.cfg['input_bag_classes'].split(',')],self.cfg['input_barcode_class'],
                self.cfg['tracking_roi_file'] or default_path(),self.cfg['track_max_age_s'],
                require_entry=self.cfg['tracking_require_roi_entry'])
        self.request=None
        self.tracking_events=SimpleQueue()
        self.tracking_event_pub=self.create_publisher(String,'/task/tracking_event',10)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.detections=self.create_publisher(DetectionArray,'/task/detections',qos_profile_sensor_data)
        self.icp=(self.create_publisher(DetectionFrame,'/task/detection_frames',LATEST_QOS)
                  if icp_transport else None)
        self.labels=self.create_publisher(BarcodeTarget,'/task/barcode_target',qos_profile_sensor_data)
        self.candidates=self.create_publisher(VisualEvidenceArray,'/task/visual_candidates',qos_profile_sensor_data)
        self.evidence=self.create_publisher(VisualEvidence,'/task/visual_evidence',qos_profile_sensor_data)
        if self.cfg['camera_input'] == 'direct':
            from .camera_input import CameraInput
            self.camera = CameraInput(self, self.enqueue, width=self.cfg['width'], height=self.cfg['height'])
            self.subscription = None
        elif self.cfg['camera_input'] == 'observation':
            self.subscription=self.create_subscription(Observation,self.cfg['input_topic'],self.enqueue,LATEST_QOS)
        else:
            raise ValueError('camera_input must be direct or observation')
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
        rgb = normalize_rgb(obs.rgb)
        if self.cfg['ignore_tracking_ids']:
            # Detection only: ByteTrack identity/confirmation is not a prerequisite.
            boxes = [dict(class_id=c, class_name=n, confidence=v, xyxy=xy, track_id='')
                     for c,n,v,xy in self.detector(rgb)]
        else:
            if self.cfg['single_target_tracking']:
                req=self.request
                self.detector.hold_target=bool(req and req.target_id and req.mode in ('locate','barcode') and stamp_ns(obs.header)<self.expiry(req))
            boxes = self.detector.track(rgb,stamp_ns(obs.header)*1e-9)
            event=getattr(self.detector,'exit_event',None)
            if event:self.tracking_events.put(event)
        for b in boxes:
            name = b['class_name']
            if name in [s.strip() for s in self.cfg['input_bag_classes'].split(',')]:
                if not self.cfg['single_target_tracking'] and b['confidence'] < self.cfg['confidence']:continue
                name = self.cfg['bag_class']
            elif name == self.cfg['input_barcode_class']:
                if b['confidence'] < self.barcode_confidence:continue
                name = self.cfg['barcode_class']
            else:
                continue
            out.boxes.append(BoundingBox(class_id=b['class_id'],class_name=name,
                confidence=b['confidence'],track_id=b['track_id'],xyxy=b['xyxy'],polygon_xy=b.get('polygon_xy',[])))
        if self.cfg['ignore_tracking_ids']:
            out.boxes = single_object_slots(out.boxes, self.cfg['bag_class'], self.cfg['barcode_class'])
        return out

    def _poll(self):
        while not self.tracking_events.empty():
            self.tracking_event_pub.publish(String(data=json.dumps(self.tracking_events.get_nowait())))
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
        if req.mode=='barcode' and not candidate_only:
            # Barcode acquisition deliberately does not depend on bag tracking/ROI.
            candidates=[b for b in frame.boxes if b.class_name==self.cfg['barcode_class']]
            out.barcode_visible=len(candidates)==1
            if out.barcode_visible:
                out.barcode_track_id=candidates[0].track_id
                try:
                    when=Time.from_msg(frame.header.stamp,clock_type=self.get_clock().clock_type)
                    bc=matrix(self.tf.lookup_transform(self.cfg['base_frame'],frame.header.frame_id,when))
                    depth,_=normalize_depth(frame.depth)
                    center,fraction=center_position(depth,frame.depth_info.p,candidates[0].xyxy,3.)
                    position=(bc@np.r_[center,1.])[:3]
                    label=BarcodeTarget(header=out.header,sequence=frame.sequence,request_id=req.request_id,
                        target_id=req.target_id,barcode_track_id=out.barcode_track_id,
                        orientation_valid=False,plane_rmse_m=float('nan'),valid_fraction=fraction)
                    label.pose.position.x,label.pose.position.y,label.pose.position.z=map(float,position)
                    label.pose.orientation.w=1.
                    self.labels.publish(label)
                except Exception as exc:self.reject(f'barcode center geometry unavailable: {exc}')
            self.evidence.publish(out)
            return out
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
            if len(selected.polygon_xy) or len(selected.xyxy):
                try:
                    lo,hi=surface_bounds(depth,frame.depth_info.p,selected.polygon_xy,bc,bbox=selected.xyxy)
                    out.bbox3d_min.x,out.bbox3d_min.y,out.bbox3d_min.z=map(float,lo)
                    out.bbox3d_max.x,out.bbox3d_max.y,out.bbox3d_max.z=map(float,hi)
                    out.bbox3d_valid=True
                except ValueError:pass
            if not candidate_only and req.mode=='barcode' and out.barcode_visible:
                position_only=self.cfg['barcode_position_only']
                if position_only:
                    center,fraction=center_position(depth,frame.depth_info.p,candidates[0].xyxy,3.)
                    pose=np.eye(4);pose[:3,3]=(bc@np.r_[center,1.])[:3]
                    rmse=float('nan')  # No plane was fitted.
                else:
                    pose,rmse,fraction=surface(depth,frame.depth_info.p,candidates[0].xyxy,3.)
                    pose=bc@pose
                label=BarcodeTarget(header=out.header,sequence=frame.sequence,request_id=req.request_id,
                    target_id=req.target_id,barcode_track_id=out.barcode_track_id,
                    orientation_valid=not position_only,plane_rmse_m=rmse,valid_fraction=fraction)
                label.pose.position.x,label.pose.position.y,label.pose.position.z=map(float,pose[:3,3])
                q=Rotation.from_matrix(pose[:3,:3]).as_quat()
                label.pose.orientation.x,label.pose.orientation.y,label.pose.orientation.z,label.pose.orientation.w=map(float,q)
                self.labels.publish(label)
        except Exception as exc:self.reject(f'visual geometry unavailable: {exc}')
        if not candidate_only:self.evidence.publish(out)
        return out


def main(args=None):run(TrackedDetectionNode,args)


def single_object_slots(boxes, bag_class, barcode_class):
    """Protocol IDs are fixed slots, not tracker identities; ambiguity is rejected."""
    bags = [b for b in boxes if b.class_name == bag_class]
    if len(bags) != 1:
        return []
    bags[0].track_id = 'single-bag'
    labels = [b for b in boxes if b.class_name == barcode_class]
    for i, label in enumerate(labels):
        label.track_id = f'frame-barcode-{i}'
    return bags + labels
