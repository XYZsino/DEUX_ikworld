"""Read-only, acquisition-matched RGB/pose overlay and localhost MJPEG viewer."""
import hashlib
import itertools
import json
import threading
import time
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from deux_task_msgs.msg import ObjectDetectionArray
from deux_task_perception.core import normalize_rgb
from deux_task_perception.perception_core import transform_matrix
from tf2_ros import Buffer, TransformListener, TransformException

HTML = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>DEUX · 실시간 물체 자세</title><style>
body{margin:0;background:#10151c;color:#e6edf5;font:16px system-ui,sans-serif}
main{max-width:1100px;margin:30px auto;padding:0 24px}header{display:flex;align-items:center;justify-content:space-between}
h1{font-size:24px}#state{padding:8px 14px;border-radius:30px;background:#253444}
img{display:block;width:100%;background:#080b10;border-radius:12px;min-height:200px}
.stats{display:flex;gap:30px;flex-wrap:wrap;padding:20px 0}.stats b{font-size:23px;display:block}.stats span,p{color:#9aabbd}
table{width:100%;border-collapse:collapse}td,th{text-align:left;padding:12px;border-bottom:1px solid #27313d}
</style><main><header><h1>DEUX · 실시간 물체 자세</h1><div id="state">연결 중</div></header>
<img src="/stream.mjpg" alt="촬영 시각이 일치하는 RGB와 물체 3D bbox">
<div class="stats"><div><b id="count">—</b><span>물체</span></div><div><b id="fps">—</b><span>표시 FPS</span></div><div><b id="age">—</b><span>촬영 후 경과</span></div><div><b>support_plane</b><span>기준 좌표계</span></div></div>
<table><thead><tr><th>ID</th><th>위치 XYZ (m)</th><th>Yaw</th><th>ICP 오차</th></tr></thead><tbody id="objects"></tbody></table>
<p>촬영 시각이 같은 RGB와 자세만 표시합니다. 모델 원점: bounds_center · yaw-only ICP · 축: X 빨강 / Y 초록 / Z 파랑.</p>
<p id="detail"></p></main><script>
async function update(){try{let s=await(await fetch('/status',{cache:'no-store'})).json();
document.getElementById('state').textContent=s.live?'실시간':s.frame?'수신 지연':'프레임 대기';
document.getElementById('state').style.background=s.live?'#175b43':'#67421f';
document.getElementById('count').textContent=s.objects?.length??'—';
document.getElementById('fps').textContent=(s.fps??0).toFixed(1);
document.getElementById('age').textContent=s.frame?(s.capture_age_s*1000).toFixed(0)+' ms':'—';
const tbody=document.getElementById('objects');tbody.replaceChildren();
for(const o of s.objects??[]){let tr=document.createElement('tr');for(const v of [o.id,o.xyz.map(x=>x.toFixed(3)).join(', '),o.yaw_deg.toFixed(1)+'°',(o.rmse_m*1000).toFixed(1)+' mm']){let td=document.createElement('td');td.textContent=v;tr.appendChild(td)}tbody.appendChild(tr)}
document.getElementById('detail').textContent=s.error||('동기화 프레임 '+s.frames+' · RGB 누락 '+s.missed);
}catch(e){document.getElementById('state').textContent='연결 끊김'}}setInterval(update,500);update();
</script></html>'''

def stamp(msg):
    return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

class Viewer(Node):
    def __init__(self):
        super().__init__('pose_rgb_viewer')
        root=Path('/home/DEUX003/DEUX_ikworld')
        import yaml
        cfg=yaml.safe_load((root/'perception.local.yaml').read_text())['icp_node']['ros__parameters']
        raw=Path(cfg['model_path']).read_bytes()
        vertices=np.array([[float(v) for v in line.split()[1:4]] for line in raw.decode().splitlines() if line.split() and line.split()[0]=='v'])
        bounds=np.array([vertices.min(0),vertices.max(0)])
        assert cfg['model_origin']=='bounds_center'
        bounds-=bounds.mean(0)
        self.corners=np.array(list(itertools.product(*zip(bounds[0],bounds[1]))))
        self.edges=[(i,j) for i in range(8) for j in range(i+1,8) if sum(a!=b for a,b in zip(self.corners[i],self.corners[j]))==1]
        assert len(self.edges)==12
        self.model_id='obj-sha256:'+hashlib.sha256(raw).hexdigest()+':bounds_center:metres'
        self.rgb=OrderedDict(); self.info=OrderedDict(); self.pending=OrderedDict()
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.condition=threading.Condition(); self.jpeg=None; self.version=0
        self.state={'frame':False,'frames':0,'missed':0,'objects':[],'error':''}
        self.times=[]
        self.subs=[self.create_subscription(Image,'/zed/zed_node/rgb/color/rect/image',self.on_rgb,qos_profile_sensor_data),
                   self.create_subscription(CameraInfo,'/zed/zed_node/rgb/color/rect/camera_info',self.on_info,qos_profile_sensor_data),
                   self.create_subscription(ObjectDetectionArray,'/task/object_detections',self.on_pose,qos_profile_sensor_data)]
        self.timer=self.create_timer(.03,self.render_pending)

    def cache(self,cache,msg):
        key=stamp(msg); cache[key]=msg
        while cache and (len(cache)>60 or next(iter(cache))<key-2_000_000_000):cache.popitem(last=False)

    def on_rgb(self,msg): self.cache(self.rgb,msg)
    def on_info(self,msg): self.cache(self.info,msg)
    def on_pose(self,msg):
        self.pending[stamp(msg)]=(msg,time.monotonic())
        while len(self.pending)>15:
            self.pending.popitem(last=False)
            self.state['missed']+=1

    def render_pending(self):
        for key,(msg,arrival) in list(self.pending.items()):
            if key not in self.rgb or key not in self.info:
                if time.monotonic()-arrival>1:
                    self.pending.pop(key); self.state['missed']+=1
                continue
            if key<=self.state.get('stamp_ns',0):
                self.pending.pop(key);continue
            try:self.render(msg,self.rgb[key],self.info[key])
            except (ValueError,TransformException) as error:
                with self.condition:self.state['error']=str(error)
                if time.monotonic()-arrival<1:continue
            self.pending.pop(key)

    def render(self,msg,rgb,info):
        if msg.model_id!=self.model_id:raise ValueError('모델 또는 원점이 일치하지 않습니다')
        if rgb.header.frame_id!=info.header.frame_id:raise ValueError('RGB와 CameraInfo 좌표계 불일치')
        tf=self.tf.lookup_transform(rgb.header.frame_id,msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
        camera_from_reference=transform_matrix([tf.translation.x,tf.translation.y,tf.translation.z],[tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w])
        projection=np.array(info.p).reshape(3,4)
        image=cv2.cvtColor(normalize_rgb(rgb),cv2.COLOR_RGB2BGR)
        objects=[]
        for obj in msg.detections:
            pose=obj.pose.pose; t=pose.position; q=pose.orientation
            reference_from_model=transform_matrix([t.x,t.y,t.z],[q.x,q.y,q.z,q.w])
            camera_from_model=camera_from_reference@reference_from_model
            def camera(points):return np.c_[points,np.ones(len(points))]@camera_from_model.T
            def project(points):
                uvw=points@projection.T
                return np.rint(uvw[:,:2]/uvw[:,2:]).astype(int)
            points=camera(self.corners)
            # Avoid invalid giant projections for boxes crossing the camera plane.
            if not np.isfinite(points).all() or np.any(points[:,2]<=.01):continue
            uv=project(points)
            for a,b in self.edges:
                good,start,end=cv2.clipLine((0,0,rgb.width,rgb.height),tuple(uv[a]),tuple(uv[b]))
                if good:cv2.line(image,start,end,(0,230,255),2,cv2.LINE_AA)
            axes=camera(np.array([[0,0,0],[.05,0,0],[0,.05,0],[0,0,.05]]))
            if np.all(axes[:,2]>.01):
                ax=project(axes)
                for i,color in enumerate([(0,0,255),(0,255,0),(255,100,50)],1):
                    good,start,end=cv2.clipLine((0,0,rgb.width,rgb.height),tuple(ax[0]),tuple(ax[i]))
                    if good:cv2.arrowedLine(image,start,end,color,2,cv2.LINE_AA,tipLength=.2)
            anchor=np.maximum(np.minimum(uv.min(0),[rgb.width-150,rgb.height-10]),[5,50])
            label=obj.track_id[-8:]+'  '+str(round(obj.trimmed_rmse_m*1000,1))+'mm'
            cv2.putText(image,label,tuple(anchor),(cv2.FONT_HERSHEY_SIMPLEX),.42,(0,0,0),3,cv2.LINE_AA)
            cv2.putText(image,label,tuple(anchor),(cv2.FONT_HERSHEY_SIMPLEX),.42,(0,230,255),1,cv2.LINE_AA)
            objects.append({'id':obj.track_id,'xyz':[t.x,t.y,t.z],'yaw_deg':float(np.degrees(np.arctan2(reference_from_model[1,0],reference_from_model[0,0]))),'rmse_m':obj.trimmed_rmse_m,'coverage':obj.target_coverage_10mm})
        age=(time.time_ns()-stamp(msg))/1e6
        cv2.rectangle(image,(0,0),(rgb.width,28),(15,20,28),-1)
        cv2.putText(image,f'3D POSE | {len(objects)} object(s) | {age:.0f} ms | yaw only',(9,19),cv2.FONT_HERSHEY_SIMPLEX,.45,(235,240,250),1,cv2.LINE_AA)
        ok,encoded=cv2.imencode('.jpg',image,[cv2.IMWRITE_JPEG_QUALITY,85])
        if not ok:raise ValueError('JPEG encoding failed')
        now=time.time(); self.times.append(now);self.times=self.times[-30:]
        with self.condition:
            self.jpeg=encoded.tobytes();self.version+=1
            self.state.update(frame=True,frames=self.version,stamp_ns=stamp(msg),sequence=msg.sequence,updated_at=now,objects=objects,error='',fps=(len(self.times)-1)/(self.times[-1]-self.times[0]) if len(self.times)>1 else 0)
            self.condition.notify_all()

    def status(self):
        with self.condition:
            result=dict(self.state)
            result['live']=time.time()-result.get('updated_at',0)<2
            result['capture_age_s']=time.time()-result.get('stamp_ns',0)/1e9
            return result

class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def do_GET(self):
        if self.path=='/': data=HTML.encode();kind='text/html; charset=utf-8'
        elif self.path=='/status':data=json.dumps(self.server.viewer.status()).encode();kind='application/json'
        elif self.path=='/snapshot.jpg':
            with self.server.viewer.condition:data=self.server.viewer.jpeg
            if data is None:self.send_error(503,'Waiting for matched pose/RGB');return
            kind='image/jpeg'
        elif self.path=='/stream.mjpg':
            self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame');self.send_header('Cache-Control','no-store');self.end_headers()
            version=-1
            try:
                while True:
                    with self.server.viewer.condition:
                        self.server.viewer.condition.wait_for(lambda:self.server.viewer.version!=version,timeout=5)
                        version=self.server.viewer.version;data=self.server.viewer.jpeg
                    if data is None:continue
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(data)).encode()+b'\r\n\r\n'+data+b'\r\n');self.wfile.flush()
            except (BrokenPipeError,ConnectionResetError):pass
            return
        else:self.send_error(404);return
        self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)

def main():
    rclpy.init();node=Viewer()
    server=ThreadingHTTPServer(('127.0.0.1',8765),Handler);server.daemon_threads=True;server.viewer=node
    threading.Thread(target=server.serve_forever,daemon=True).start()
    print('Pose RGB viewer: http://127.0.0.1:8765',flush=True)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:server.shutdown();server.server_close();node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
