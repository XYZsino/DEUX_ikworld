"""Selected-instance visible-surface 3D bounds, projected into its exact RGB frame.

These are depth-derived bounds of the visible surface, NOT a model amodal box,
ICP full pose, ground truth or a success predicate. No robot command is emitted.
"""
from collections import deque
from io import BytesIO
import time
import numpy as np
from PIL import Image, ImageDraw
from deux_task_perception.core import normalize_rgb, normalize_depth, validate_bundle, stamp_ns
from deux_task_perception.perception_core import transform_matrix
from deux_task_perception.visible_geometry import surface_bounds, project_bounds


def corners_for(obs, detections, target_id, base_frame='base_link'):
    if not target_id:raise ValueError('선택된 대상 없음')
    if detections is None or (detections.sequence!=obs.sequence or stamp_ns(detections.header)!=stamp_ns(obs.header)
            or detections.header.frame_id!=obs.header.frame_id):
        raise ValueError('같은 영상의 검출 결과 대기')
    selected=[b for b in detections.boxes if b.track_id==target_id and b.class_name=='bread_bag']
    if len(selected)!=1:raise ValueError('선택 대상이 현재 영상에 없음')
    box=selected[0];polygon=np.asarray(box.polygon_xy,dtype=float)
    if polygon.size<6 or polygon.size%2 or not np.isfinite(polygon).all():
        raise ValueError('선택 대상 분할 마스크 없음')
    w,h=obs.rgb.width,obs.rgb.height
    validate_bundle(obs.rgb,obs.depth,obs.rgb_info,obs.depth_info,w,h,20_000_000)
    if stamp_ns(obs.header)!=stamp_ns(obs.rgb.header) or obs.header.frame_id!=obs.rgb.header.frame_id:
        raise ValueError('영상 헤더 불일치')
    tf=obs.base_from_camera
    if (not obs.camera_transform_valid or tf.header.frame_id!=base_frame or tf.child_frame_id!=obs.header.frame_id
            or abs(stamp_ns(tf.header)-stamp_ns(obs.header))>20_000_000):
        raise ValueError('영상 시점 카메라 TF 없음')
    t,q=tf.transform.translation,tf.transform.rotation
    bc=transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
    depth,_=normalize_depth(obs.depth)
    lo,hi=surface_bounds(depth,obs.rgb_info.p,polygon,bc)
    pixels=project_bounds(lo,hi,bc,obs.rgb_info.p)
    extent=hi-lo
    if np.max(np.abs(pixels))>max(w,h)*4:raise ValueError('투영 범위 초과')
    return pixels,extent


def render_rgb(obs,detections,target_id,base_frame='base_link'):
    image=Image.fromarray(normalize_rgb(obs.rgb));reason='';extent=None
    roi=None
    if (detections is not None and detections.sequence==obs.sequence
            and detections.header==obs.header and detections.selection_roi_valid):
        bounds=np.asarray(detections.selection_roi_xyxy)
        if np.isfinite(bounds).all() and np.all(bounds[2:]>bounds[:2]):
            roi=bounds.tolist();draw=ImageDraw.Draw(image)
            draw.rectangle(roi,outline='#ffcc66',width=2)
            draw.text((roi[0]+5,roi[1]+5),'Pick ROI',fill='#ffcc66')
    try:
        pixels,extent=corners_for(obs,detections,target_id,base_frame)
        draw=ImageDraw.Draw(image)
        for i in range(8):
            for axis in (1,2,4):
                j=i^axis
                if j>i:draw.line([tuple(pixels[i]),tuple(pixels[j])],fill='#63ffd4',width=3)
        x=max(4,min(image.width-230,int(pixels[:,0].min())));y=max(4,int(pixels[:,1].min())-24)
        draw.rectangle((x,y,x+226,y+21),fill='#102b30')
        draw.text((x+5,y+4),f'{target_id} | visible 3D bounds',fill='white')
    except ValueError as exc:reason=str(exc)
    output=BytesIO();image.save(output,format='JPEG',quality=85)
    return output.getvalue(),dict(target_id=target_id,box_valid=extent is not None,selection_roi_xyxy=roi,
        dimensions_m=extent.tolist() if extent is not None else [],reason=reason,
        acquired_s=stamp_ns(obs.header)*1e-9,sequence=obs.sequence,frame_id=obs.header.frame_id,
        method='registered depth + selected mask; visible surface bounds')


class RgbPreview:
    def __init__(self,node):
        from rclpy.qos import qos_profile_sensor_data
        from deux_task_msgs.msg import Observation,DetectionArray
        self.node=node;self.observations=deque(maxlen=8);self.detections=deque(maxlen=8)
        self.jpeg=None;self.metadata={};self.published=0.;self.key=None
        self.subs=[node.create_subscription(Observation,node.get_parameter('observation_topic').value,
                     lambda m:self.observations.append((time.monotonic(),m)),qos_profile_sensor_data),
                   node.create_subscription(DetectionArray,node.get_parameter('detections_topic').value,
                     lambda m:self.detections.append((time.monotonic(),m)),qos_profile_sensor_data)]

    def update(self):
        now=time.monotonic()
        if not self.observations or now-self.observations[-1][0]>.8:return
        _,obs=self.observations[-1];target=self.node.status.get('target_id','');detection=None
        # Show the selection ROI even before a target has been selected.
        for received,candidate in reversed(self.observations):
            if now-received>.8 or stamp_ns(obs.header)-stamp_ns(candidate.header)>500_000_000:continue
            match=next((d for when,d in reversed(self.detections) if now-when<.8 and d.sequence==candidate.sequence
                        and stamp_ns(d.header)==stamp_ns(candidate.header) and d.header.frame_id==candidate.header.frame_id),None)
            if match is not None:obs,detection=candidate,match;break
        key=(stamp_ns(obs.header),obs.sequence,obs.header.frame_id,target,detection is not None)
        if key==self.key:return
        try:jpeg,metadata=render_rgb(obs,detection,target,self.node.get_parameter('camera_base_frame').value)
        except (ValueError,TypeError,IndexError) as exc:
            self.metadata={'reason':str(exc),'box_valid':False};self.jpeg=None;return
        self.key=key;self.jpeg=jpeg;self.metadata=metadata;self.published=now

    def snapshot(self):
        return self.jpeg if (time.monotonic()-self.published<.8 and
                            self.metadata.get('target_id')==self.node.status.get('target_id','')) else None
