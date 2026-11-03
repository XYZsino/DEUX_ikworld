"""RGB preview with acquisition-matched ICP pose axes; no depth or box input."""
from collections import deque
from io import BytesIO
import time
import numpy as np
from PIL import Image, ImageDraw
from deux_task_perception.core import normalize_rgb, stamp_ns
from deux_task_perception.perception_core import transform_matrix


def icp_axes_for(obs, objects, target_id, base_frame='base_link', length=.08, allow_historical=False):
    if objects is None:
        raise ValueError('ICP pose unavailable')
    if (objects.header.frame_id != base_frame or
            (not allow_historical and (objects.sequence != obs.sequence or
             stamp_ns(objects.header) != stamp_ns(obs.header))) or
            stamp_ns(objects.header) > stamp_ns(obs.header)):
        raise ValueError('ICP pose/image acquisition mismatch')
    selected = [d for d in objects.detections if d.track_id == target_id]
    if len(selected) != 1:
        raise ValueError('selected ICP pose unavailable')
    tf=obs.base_from_camera
    if (not obs.camera_transform_valid or tf.header.frame_id != base_frame
            or tf.child_frame_id != obs.header.frame_id
            or abs(stamp_ns(tf.header)-stamp_ns(obs.header)) > 20_000_000):
        raise ValueError('ICP axes camera TF unavailable')
    t,q=tf.transform.translation,tf.transform.rotation
    bc=transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
    pose=selected[0].pose.pose;t,q=pose.position,pose.orientation
    bm=transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
    local=np.vstack((np.zeros(3),np.eye(3)*length))
    cm=np.linalg.inv(bc)@bm
    points=local@cm[:3,:3].T+cm[:3,3]
    if not np.isfinite(points).all() or np.any(points[:,2]<=.01):
        raise ValueError('ICP axes behind camera')
    projection=np.asarray(obs.rgb_info.p).reshape(3,4)
    pixels=np.column_stack((points,np.ones(4)))@projection.T
    pixels=pixels[:,:2]/pixels[:,2,None]
    if not np.isfinite(pixels).all() or np.max(np.abs(pixels))>max(obs.rgb.width,obs.rgb.height)*4:
        raise ValueError('ICP axes projection outside range')
    return pixels


def grasp_pixels(obs, preview, target_id, base_frame='base_link'):
    """Project a future feasible IK endpoint; never feed it into perception."""
    if not preview or not preview.get('feasible') or preview.get('expired') or preview.get('target_id')!=target_id:
        return None
    tf=obs.base_from_camera
    if not obs.camera_transform_valid or tf.header.frame_id!=base_frame or tf.child_frame_id!=obs.header.frame_id or abs(stamp_ns(tf.header)-stamp_ns(obs.header))>20_000_000:
        return None
    t,q=tf.transform.translation,tf.transform.rotation
    camera_from_base=np.linalg.inv(transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w]))
    grasp=np.asarray(preview['alignment_pose'] if preview.get('preview_kind')=='position_first_alignment' else preview['grasp_pose']);approach=np.asarray(preview['approach_pose'])
    points=np.vstack([grasp[:3,3],approach[:3,3],grasp[:3,3]+grasp[:3,:3].T*.05])
    camera=points@camera_from_base[:3,:3].T+camera_from_base[:3,3]
    if not np.isfinite(camera).all() or np.any(camera[:,2]<=.01):return None
    pixels=np.column_stack([camera,np.ones(len(camera))])@np.asarray(obs.rgb_info.p).reshape(3,4).T
    pixels=pixels[:,:2]/pixels[:,2,None]
    if not np.isfinite(pixels).all() or np.max(np.abs(pixels))>max(obs.rgb.width,obs.rgb.height)*4:return None
    return pixels



def intercept_pixels(obs,window,base_frame='base_link'):
    if not window or window.get('frame')!=base_frame:return None
    tf=obs.base_from_camera
    if (not obs.camera_transform_valid or tf.header.frame_id!=base_frame
        or tf.child_frame_id!=obs.header.frame_id or abs(stamp_ns(tf.header)-stamp_ns(obs.header))>20_000_000):return None
    points=np.asarray(window.get('polygon',[]),float)
    if points.shape!=(4,3) or not np.isfinite(points).all():return None
    encounter=window.get('encounter_position') if window.get('validated') else None
    if encounter is not None:
        point=np.asarray(encounter,float)
        if point.shape!=(3,) or not np.isfinite(point).all():return None
        points=np.vstack([points,point])
    t,q=tf.transform.translation,tf.transform.rotation
    cb=np.linalg.inv(transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w]))
    cam=points@cb[:3,:3].T+cb[:3,3]
    if np.any(cam[:,2]<=.01):return None
    uv=np.column_stack([cam,np.ones(len(cam))])@np.asarray(obs.rgb_info.p).reshape(3,4).T
    uv=uv[:,:2]/uv[:,2,None]
    if not np.isfinite(uv).all() or np.max(abs(uv))>max(obs.rgb.width,obs.rgb.height)*4:return None
    return uv

def render_rgb(obs,detections,target_id,base_frame='base_link',objects=None,evidence=None,allow_historical=False,tracking_roi=None,grasp_preview=None,intercept_window=None,via_presence_roi=None):
    image=Image.fromarray(normalize_rgb(obs.rgb))
    axes_valid=False;axes_reason=''
    try:
        axes=icp_axes_for(obs,objects,target_id,base_frame,allow_historical=allow_historical)
        draw=ImageDraw.Draw(image)
        for index,color in enumerate(('#ff4040','#40e060','#4080ff'),1):
            draw.line([tuple(axes[0]),tuple(axes[index])],fill=color,width=3)
            draw.text(tuple(axes[index]+3),'XYZ'[index-1],fill=color,stroke_width=1,stroke_fill='#101820')
        axes_valid=True
    except ValueError as exc:axes_reason=str(exc)
    draw=ImageDraw.Draw(image)
    if tracking_roi:
        poly=np.asarray(tracking_roi['points'])*[obs.rgb.width/tracking_roi['width'],obs.rgb.height/tracking_roi['height']]
        vertices=[tuple(x) for x in poly];draw.line(vertices+[vertices[0]],fill='#00ff90',width=2)
    if via_presence_roi:
        poly=np.asarray(via_presence_roi['points'])*[obs.rgb.width/via_presence_roi['width'],obs.rgb.height/via_presence_roi['height']]
        vertices=[tuple(x) for x in poly]
        draw.line(vertices+[vertices[0]],fill='#ff70ff',width=3)
        draw.text((max(0,poly[:,0].min()),max(0,poly[:,1].min()-14)),'VIA BAG ROI',fill='#ff70ff',stroke_width=1,stroke_fill='#101820')
    if detections is not None and 0 <= (stamp_ns(obs.header)-stamp_ns(detections.header))*1e-9 <= .25:
        for box in detections.boxes:
            if not box.track_id.startswith('single-'):continue
            draw.rectangle(tuple(box.xyxy),outline='#ffff00',width=2)
            draw.text((box.xyxy[0],max(0,box.xyxy[1]-12)),box.track_id,fill='#ffff00')
    window_pixels=intercept_pixels(obs,intercept_window,base_frame)
    if window_pixels is not None:
        vertices=[tuple(p) for p in window_pixels[:4]]
        draw.line(vertices+[vertices[0]],fill='#ffb040',width=3)
        draw.text(vertices[0],'GRASP WINDOW (candidate)',fill='#ffb040',stroke_width=1,stroke_fill='#101820')
        if len(window_pixels)>4:
            x,y=window_pixels[4];draw.ellipse((x-6,y-6,x+6,y+6),outline='#00ffff',width=3)
            draw.text((x+8,y),'MEET',fill='#00ffff',stroke_width=1,stroke_fill='#101820')
    points=grasp_pixels(obs,grasp_preview,target_id,base_frame)
    if points is not None:
        g,a=points[:2]
        tf=obs.base_from_camera;t,q=tf.transform.translation,tf.transform.rotation
        cb=np.linalg.inv(transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w]))
        projection=np.asarray(obs.rgb_info.p).reshape(3,4)
        for segment in grasp_preview.get('trace',[]):
            xyz=np.asarray(segment['points']);camera=xyz@cb[:3,:3].T+cb[:3,3]
            uv=np.column_stack([camera,np.ones(len(camera))])@projection.T
            good=(camera[:,2]>.01)&np.isfinite(uv).all(axis=1)&(abs(uv[:,2])>1e-9)
            pixels=np.full((len(camera),2),np.nan);pixels[good]=uv[good,:2]/uv[good,2,None]
            color={'Approach':'#00ffff','Descend':'#ff70ff','Close':'#ffe060','Align':'#ffb040'}[segment['stage']]
            for first,second in zip(pixels[:-1],pixels[1:]):
                if np.isfinite([first,second]).all() and np.max(np.abs([first,second]))<100000:draw.line([tuple(first),tuple(second)],fill=color,width=3)
            if segment['stage']=='Close' and np.isfinite(pixels[-1]).all():
                x,y=pixels[-1]
                if 0<=x<image.width and 0<=y<image.height:
                    draw.ellipse((x-6,y-6,x+6,y+6),outline=color,width=3);draw.text((x+8,y-9),'C',fill=color,stroke_width=1,stroke_fill='#101820')
        draw.line([tuple(a),tuple(g)],fill='#ff70ff',width=2)
        for p,label,color in ((g,'R' if grasp_preview.get('preview_kind')=='position_first_alignment' else 'G','#ffb040' if grasp_preview.get('preview_kind')=='position_first_alignment' else '#ff70ff'),(a,'A','#00ffff')):
            x,y=p;draw.ellipse((x-6,y-6,x+6,y+6),outline=color,width=3)
            draw.text((x+8,y-9),label,fill=color,stroke_width=1,stroke_fill='#101820')
        for p,color in zip(points[2:],('#ff4040','#40e060','#4080ff')):
            draw.line([tuple(g),tuple(p)],fill=color,width=2)
    output=BytesIO();image.save(output,format='JPEG',quality=85)
    return output.getvalue(),dict(via_presence_roi=via_presence_roi,via_presence_roi_visible=via_presence_roi is not None,intercept_window_visible=window_pixels is not None,grasp_preview_visible=points is not None,icp_axes_valid=axes_valid,icp_axes_reason=axes_reason,
        icp_axes_held=bool(axes_valid and objects is not None and stamp_ns(objects.header)!=stamp_ns(obs.header)),
        icp_pose_age_s=max(0.,(stamp_ns(obs.header)-stamp_ns(objects.header))*1e-9) if objects is not None else None,
        icp_axis_length_m=.08,target_id=target_id,box_valid=False,selection_roi_xyxy=None,
        dimensions_m=[],reason=axes_reason,acquired_s=stamp_ns(obs.header)*1e-9,
        sequence=obs.sequence,frame_id=obs.header.frame_id,method='ICP pose axes only')



def predict_preview(objects,target_id,stamp,max_horizon=.75):
    """Visualization only; never publishes predicted measurements to control."""
    from copy import deepcopy
    from scipy.spatial.transform import Rotation
    result=deepcopy(objects);selected=[d for d in result.detections if d.track_id==target_id]
    if len(selected)!=1:raise ValueError('preview target unavailable')
    age=(stamp.sec*1e9+stamp.nanosec-stamp_ns(objects.header))/1e9
    if age<0:raise ValueError('preview source is newer than image')
    d=selected[0];dt=min(age,max_horizon) if d.velocity_valid else 0.
    if dt:
        p=d.pose.pose.position;v=d.velocity.linear
        p.x+=v.x*dt;p.y+=v.y*dt;p.z+=v.z*dt
        q=d.pose.pose.orientation
        rotation=Rotation.from_rotvec([0.,0.,d.velocity.angular.z*dt])*Rotation.from_quat([q.x,q.y,q.z,q.w])
        q.x,q.y,q.z,q.w=map(float,rotation.as_quat())
    result.header.stamp=deepcopy(stamp);result.detections=selected
    return result,dict(predicted_preview=True,prediction_age_s=age,prediction_dt_s=dt,prediction_velocity_valid=bool(d.velocity_valid),prediction_stale=age>max_horizon)


class RgbPreview:
    def __init__(self,node):
        from rclpy.qos import qos_profile_sensor_data
        from deux_task_msgs.msg import Observation,ObjectDetectionArray,DetectionArray
        self.node=node;self.observations=deque(maxlen=24);self.objects=deque(maxlen=8)
        self.jpeg=None;self.metadata={};self.published=0.;self.key=None;self.cached_pose=None;self.cached_target=''
        self.subs=[node.create_subscription(ObjectDetectionArray,'/task/object_detections',
                     lambda m:self.objects.append((time.monotonic(),m)),qos_profile_sensor_data)]
        self.tracks=deque(maxlen=8)
        self.subs.append(node.create_subscription(DetectionArray,node.get_parameter('detections_topic').value,
            lambda m:self.tracks.append(m),qos_profile_sensor_data))
        source = node.get_parameter('camera_input').value
        if source == 'auto':
            source = 'direct' if node.deployment == 'real' else 'observation'
        if source == 'direct':
            from deux_task_perception.camera_input import RgbCameraInput
            from tf2_ros import Buffer, TransformListener
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf, node)
            self.camera = RgbCameraInput(node, self.receive_camera)
        elif source == 'observation':
            self.subs.append(node.create_subscription(Observation,node.get_parameter('observation_topic').value,
                lambda m:self.observations.append((time.monotonic(),m)),qos_profile_sensor_data))
        else:
            raise ValueError('camera_input must be auto, direct or observation')

    def receive_camera(self, frame):
        from rclpy.time import Time
        from tf2_ros import TransformException
        age = self.node.get_clock().now().nanoseconds - stamp_ns(frame.header)
        if not 0 <= age <= 500_000_000:
            return
        try:
            frame.base_from_camera = self.tf.lookup_transform(
                self.node.get_parameter('camera_base_frame').value, frame.header.frame_id,
                Time.from_msg(frame.header.stamp, clock_type=self.node.get_clock().clock_type))
            frame.camera_transform_valid = True
        except TransformException:
            pass
        self.observations.append((time.monotonic(),frame))

    def tracking_view(self):
        return ((self.node.perception_only() if hasattr(self.node,'perception_only') else False)
                or getattr(self.node,'status',{}).get('workflow')=='MovingUpperTrial')

    def update(self):
        now=time.monotonic()
        if not self.observations or now-self.observations[-1][0]>.8:return
        _,obs=self.observations[-1]
        preview=self.node.perception_only() if hasattr(self.node,'perception_only') else False
        tracking_view=self.tracking_view()
        if tracking_view and not self.node.icp_preview.get('tracking',False):
            self.cached_pose=None;self.cached_target='';self.objects.clear()
        status=self.node.status
        target=(self.node.icp_preview.get('target_id','') if tracking_view else status.get('target_id',''))
        # Static pre-grasp ICP is no longer the object's position after pickup.
        show_static_axes=status.get('state','') in ('EstimatePose','Approach','Descend','Close')
        if not tracking_view and (not target or target!=self.cached_target or not show_static_axes):
            self.cached_pose=None;self.cached_target=''
            if not target or not show_static_axes:self.objects.clear()
        base=self.node.get_parameter('camera_base_frame').value
        # Visualization-only cache: never republishes or relabels a measurement.
        for received,pose in reversed(self.objects):
            if not any(d.track_id==target for d in pose.detections):continue
            try:icp_axes_for(obs,pose,target,base,allow_historical=True)
            except ValueError:continue
            self.cached_pose=pose;self.cached_target=target
            break
        objects=self.cached_pose if target==self.cached_target else None
        prediction={}
        if tracking_view and objects is not None:
            try:objects,prediction=predict_preview(objects,target,obs.header.stamp)
            except ValueError:objects=None
        key=(stamp_ns(obs.header),obs.sequence,obs.header.frame_id,target,
             stamp_ns(objects.header) if objects is not None else None)
        if key==self.key:return
        try:
            from deux_task_perception.tracking_roi import read_roi
            roi=read_roi(self.node.get_parameter('tracking_roi_file').value) if self.node.has_parameter('tracking_roi_file') else None
            via_roi=read_roi(self.node.get_parameter('scan_presence_roi_file').value) if self.node.has_parameter('scan_presence_roi_file') else None
            tracks=next((m for m in reversed(self.tracks) if stamp_ns(m.header)<=stamp_ns(obs.header)),None)
            grasp=getattr(self.node,'grasp_preview',{}) if preview and now-getattr(self.node,'grasp_preview_received',0)<2. else None
            window=getattr(self.node,'intercept_window',{}) if now-getattr(self.node,'intercept_window_received',0)<2. else None
            jpeg,metadata=render_rgb(obs,tracks,target,base,objects,allow_historical=True,tracking_roi=roi,grasp_preview=grasp,intercept_window=window,via_presence_roi=via_roi)
        except (ValueError,TypeError,IndexError) as exc:
            self.metadata={'reason':str(exc),'box_valid':False};self.jpeg=None;return
        metadata.update(prediction)
        self.key=key;self.jpeg=jpeg;self.metadata=metadata;self.published=now

    def snapshot(self):
        return self.jpeg if (time.monotonic()-self.published<.8 and
                            self.metadata.get('target_id')==((self.node.icp_preview.get('target_id','') if self.tracking_view() else self.node.status.get('target_id','')))) else None
