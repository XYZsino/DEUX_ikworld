"""A single ROS executor owns commands; HTTP threads only enqueue and read snapshots."""
import json
import os
import queue
import secrets
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib.resources import files
from urllib.parse import urlsplit

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy
from deux_task_msgs.action import RunTask, MoveToReady
from deux_task_msgs.msg import TaskStatus
from std_msgs.msg import String


class TaskGui(Node):
    def __init__(self, **kwargs):
        super().__init__('task_gui', **kwargs)
        for key, value in dict(host='127.0.0.1', port=8080, deployment='sim',
                               action_name='/task/run', status_topic='/task/status',
                               ready_action_name='/task/move_to_ready',
                               observation_topic='/task/observation', detections_topic='/task/detections',
                               camera_base_frame='base_link', camera_input='auto').items():
            self.declare_parameter(key, value)
        from deux_task_perception.tracking_roi import default_path
        self.declare_parameter('tracking_roi_file',default_path())
        self.deployment = self.get_parameter('deployment').value
        if self.deployment not in ('sim', 'real'):
            raise ValueError('deployment must be sim or real (display label only)')
        self.token = os.environ.get('DEUX_GUI_TOKEN') or secrets.token_urlsafe(32)
        self.commands = queue.Queue(maxsize=1)
        self.lock = threading.Lock()
        self.view = {}
        self.frame = None; self.frame_received = 0.; self.frame_time = 0.
        self.status = {}; self.received = 0.; self.sequence = 0
        self.history = deque(maxlen=100)
        self.ik_diagnostic=None
        self.ik_sub=self.create_subscription(String,'/task/ik_diagnostic',self.on_ik_diagnostic,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.icp_preview={};self.icp_preview_received=0.
        self.intercept_window={};self.intercept_window_received=0.
        self.intercept_window_sub=self.create_subscription(String,'/task/intercept_window',self.on_intercept_window,10)
        self.grid_preview={};self.grid_preview_received=0.
        self.grid_preview_sub=self.create_subscription(String,'/task/grid_grasp/preview',self.on_grid_preview,10)
        self.grasp_preview={};self.grasp_preview_received=0.
        self.grasp_preview_sub=self.create_subscription(String,'/task/grasp_timing_preview',self.on_grasp_preview,10)
        self.icp_preview_sub=self.create_subscription(String,'/task/icp_preview_status',self.on_icp_preview,10)
        self.time_candidates = None
        self.motion_preview=None;self.preview_received=0.
        self.preview_sub=self.create_subscription(String,'/task/motion_preview',self.on_preview,
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.decision_pub=self.create_publisher(String,'/task/motion_decision',10)
        self.candidate_sub = self.create_subscription(String,'/task/grasp_candidates',self.on_candidates,10)
        self.goal_handle = None; self.sending = False; self.canceling = False
        self.restart_target = None; self.await_idle_after = None
        self.ready_pending = False
        self.restart_deadline = 0.
        self.message = '작업 서버 연결 대기'; self.result = None
        from .rgb_view import RgbPreview
        self.rgb = RgbPreview(self)
        self.rgb_timer = self.create_timer(.2, self.rgb.update, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.client = ActionClient(self, RunTask, self.get_parameter('action_name').value)
        self.ready_client = ActionClient(self, MoveToReady, self.get_parameter('ready_action_name').value)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.sub = self.create_subscription(TaskStatus, self.get_parameter('status_topic').value, self.on_status, qos)
        self.timer = self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.http = ThreadingHTTPServer((self.get_parameter('host').value, self.get_parameter('port').value), handler(self))
        self.http.daemon_threads = True
        self.thread = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.thread.start()
        port = self.http.server_address[1]
        self.get_logger().info(f'GUI: http://127.0.0.1:{port}/#token={self.token}')

    def on_ik_diagnostic(self,msg):
        try:
            data=json.loads(msg.data)
            if isinstance(data,dict) and all(k in data for k in ('id','current','target_pose','object_pose','grasp_pose')):
                self.ik_diagnostic=data
        except (ValueError,TypeError):pass

    def on_preview(self,msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict) or not all(k in data for k in ('request_id','plan_id','digest','status','current','target','frames')):return
            if len(data['frames'])>60:return
            self.motion_preview=data;self.preview_received=time.monotonic()
        except (ValueError,TypeError):return

    def on_candidates(self, msg):
        try:
            data=json.loads(msg.data)
            if not isinstance(data,dict) or not data.get('time_search'):return
            if data.get('time_bins_s') and len(data.get('candidates',[]))<=300:
                self.time_candidates=data
            elif self.time_candidates and data.get('target_id')==self.time_candidates.get('target_id'):
                self.time_candidates=dict(self.time_candidates,latest_validation=data['scope'],
                    validation_ok=all(row.get('feasible',False) for row in data.get('candidates',[])))
        except (ValueError,TypeError,KeyError):return

    def on_status(self, msg):
        self.received = time.monotonic(); self.sequence += 1
        data = {k: getattr(msg, k) for k in ('state','target_id','reason','active','grasp_attempt','scan_attempt','ik_calls','workflow')}
        if data.get('state') != self.status.get('state'):
            self.history.append(dict(state=msg.state, at=time.time()))
        if msg.state=='Idle' or (self.status.get('target_id') and self.status.get('target_id')!=msg.target_id):
            self.time_candidates=None
        self.status = data

    def on_intercept_window(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        if isinstance(data,dict):self.intercept_window=data;self.intercept_window_received=time.monotonic()

    def on_grid_preview(self, msg):
        try: data = json.loads(msg.data)
        except ValueError: return
        if isinstance(data, dict):
            self.grid_preview = data; self.grid_preview_received = time.monotonic()

    def on_grasp_preview(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        if isinstance(data,dict):self.grasp_preview=data;self.grasp_preview_received=time.monotonic()

    def on_icp_preview(self,msg):
        try:data=json.loads(msg.data)
        except ValueError:return
        if isinstance(data,dict):self.icp_preview=data;self.icp_preview_received=time.monotonic()

    def perception_only(self):
        # Dedicated scheduled-motion server owns this explicitly requested real trial.
        if self.status.get('workflow')=='MovingUpperTrial' and time.monotonic()-self.received<1. and self.ready_client.server_is_ready():return False
        return bool(self.icp_preview.get('perception_only') and time.monotonic()-self.icp_preview_received<2.)

    def snapshot(self):
        with self.lock:
            return dict(self.view)

    def set_frame(self, jpeg, simulation_time):
        # Called by the simulation executor. HTTP threads never touch MuJoCo.
        with self.lock:
            self.frame = jpeg; self.frame_time = simulation_time
            self.frame_received = time.monotonic()

    def frame_snapshot(self):
        with self.lock:
            return self.frame if time.monotonic()-self.frame_received < 3. else None

    def connected(self):
        return self.perception_only() or (time.monotonic()-self.received < 2. and (self.client.server_is_ready() or self.ready_client.server_is_ready()))

    def start(self, target):
        if self.perception_only():self.message='인식 전용 테스트: 로봇 실행 비활성';return
        if not self.client.server_is_ready():self.message='작업 서버 비활성: 준비자세 시험만 가능합니다';return
        self.sending = True; self.result = None; self.message = '시작 요청 중'
        try:
            self.client.send_goal_async(RunTask.Goal(target_id=target)).add_done_callback(self.accepted)
        except Exception as exc:
            self.sending = False; self.restart_target = None; self.message = str(exc)

    def start_ready(self):
        if self.perception_only():self.message='인식 전용 테스트: 로봇 실행 비활성';return
        self.sending=True;self.result=None;self.message='준비자세 이동 요청 중'
        try:
            self.ready_client.send_goal_async(MoveToReady.Goal()).add_done_callback(self.accepted)
        except Exception as exc:
            self.sending=False;self.ready_pending=False;self.message=str(exc)

    def accepted(self, future):
        self.sending = False
        try:
            handle = future.result()
            if not handle.accepted:
                self.canceling = False; self.restart_target = None
                self.ready_pending = False
                self.message = '작업 서버가 시작 요청을 거부했습니다'; return
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(self.finished)
            if self.canceling:
                self.cancel_goal()
            else:
                self.message = '실행 중'
        except Exception as exc:
            self.message = f'시작 응답 오류: {exc}'
            self.restart_target = None
            self.ready_pending = False

    def cancel_goal(self):
        self.canceling = True; self.message = '중단 완료 대기'
        handle = self.goal_handle
        handle.cancel_goal_async().add_done_callback(lambda f: self.canceled(f, handle))

    def canceled(self, future, handle):
        if self.goal_handle is not handle:
            return
        try:
            if not future.result().goals_canceling and self.goal_handle is not None:
                self.restart_target = None
                self.ready_pending = False
                self.message = '취소가 수락되지 않았습니다. 작업 결과 확인 중'
        except Exception as exc:
            self.restart_target = None; self.message = f'취소 응답 오류: {exc}'
            self.ready_pending = False
        # Acceptance is not completion. Keep ownership until the terminal result.

    def finished(self, future):
        try:
            response = future.result()
            self.result = dict(success=response.result.success, reason=response.result.reason,
                               barcode=getattr(response.result,'barcode',''), status=response.status)
            self.message = response.result.reason
            self.goal_handle = None; self.canceling = False
            self.await_idle_after = self.sequence
        except Exception as exc:
            # Unknown outcome must never trigger a replacement goal.
            self.restart_target = None; self.message = f'종료 확인 실패: {exc}'
            self.ready_pending = False

    def command(self, command, target):
        if command in ('motion_approve','motion_reject'):
            p=self.motion_preview
            if (not p or p['request_id']!=target or p['status']!='pending'
                    or time.monotonic()-self.preview_received>1.):
                self.message='승인 대상을 다시 확인하세요. 계획이 만료되거나 변경됐습니다';return
            if command=='motion_approve' and not p.get('approvable',False):
                self.message='이동 컨베이어 예약 계획은 수동 승인 대기 후 실행할 수 없습니다';return
            self.decision_pub.publish(String(data=json.dumps(dict(request_id=p['request_id'],
                plan_id=p['plan_id'],digest=p['digest'],decision=command.removeprefix('motion_')))))
            self.motion_preview=dict(p,status='decision_sent')
            self.message='실행 승인 요청 전송' if command=='motion_approve' else '동작 거절 요청 전송'
            return
        owned = self.goal_handle is not None or self.sending
        if command == 'stop':
            self.restart_target = None
            self.ready_pending = False
            if owned:
                if not self.canceling:
                    self.canceling = True
                    if self.goal_handle is not None: self.cancel_goal()
                self.message = '중단 완료 대기'
            else:
                self.message = '이 GUI에서 실행한 작업이 없습니다'
            return
        if not self.connected():
            self.message = '작업 서버 또는 상태 연결이 끊겨 요청하지 않았습니다'; return
        if command == 'ready':
            if not self.ready_client.server_is_ready():
                self.message='준비자세 서버 연결 대기';return
            if owned:
                self.restart_target=None;self.ready_pending=True
                self.restart_deadline=time.monotonic()+15.
                if not self.canceling:
                    self.canceling=True
                    if self.goal_handle is not None:self.cancel_goal()
                self.message='작업 중단 후 준비자세 이동 대기';return
            if self.status.get('active') or self.await_idle_after is not None:
                self.message='작업 종료 확인 후 다시 시도하세요';return
            self.start_ready();return
        if command == 'restart' and owned:
            self.ready_pending = False
            self.restart_target = target
            self.restart_deadline = time.monotonic()+15.
            if not self.canceling:
                self.canceling = True
                if self.goal_handle is not None: self.cancel_goal()
            return
        if owned or self.status.get('active') or self.await_idle_after is not None:
            self.message = '작업 종료 확인 후 다시 시도하세요'; return
        self.start(target)

    def tick(self):
        if self.ready_pending and (not self.connected() or not self.ready_client.server_is_ready() or time.monotonic()>self.restart_deadline):
            self.ready_pending=False;self.message='연결 또는 종료 확인 지연으로 준비자세 예약을 해제했습니다'
        if self.restart_target is not None and (not self.connected() or time.monotonic() > self.restart_deadline):
            self.restart_target = None
            self.message = '연결 또는 종료 확인 지연으로 재시작 예약을 해제했습니다'
        if self.await_idle_after is not None and self.sequence > self.await_idle_after and not self.status.get('active') and self.connected():
            self.await_idle_after = None
            if self.ready_pending:
                self.ready_pending=False;self.start_ready()
            elif self.restart_target is not None:
                target = self.restart_target; self.restart_target = None
                self.start(target)
        try:
            self.command(*self.commands.get_nowait())
        except queue.Empty:
            pass
        owned = self.goal_handle is not None or self.sending
        with self.lock:
            self.view = dict(deployment=self.deployment, connected=self.connected(), status=self.status,
                owned=owned, pending=self.sending, stopping=self.canceling,
                awaiting_idle=self.await_idle_after is not None, restart_pending=self.restart_target is not None,
                message=self.message, result=self.result, history=list(self.history))
            self.view.update(ready_pending=self.ready_pending,ready_available=self.ready_client.server_is_ready())
            self.view.update(frame_available=self.frame is not None,
                             frame_fresh=time.monotonic()-self.frame_received < 3.,
                             simulation_time=self.frame_time)
            self.view['motion_preview'] = dict(self.motion_preview,fresh=time.monotonic()-self.preview_received<1.) if self.motion_preview else None
            self.view['ik_diagnostic']=self.ik_diagnostic
            self.view['time_candidates'] = self.time_candidates
            self.view['perception_only']=self.perception_only()
            self.view['intercept_window']=dict(self.intercept_window) if time.monotonic()-self.intercept_window_received<2. else {}
            self.view['grid_preview']=dict(self.grid_preview, fresh=time.monotonic()-self.grid_preview_received<2.)
            self.view['icp_preview']=dict(self.icp_preview)
            self.view['grasp_preview']=dict(self.grasp_preview) if time.monotonic()-self.grasp_preview_received<2. else {}
            if self.perception_only():
                self.view['status']=dict(state='PerceptionTest',active=False,target_id=self.icp_preview.get('target_id',''),reason='인식 전용 · 로봇 동작 없음')

            self.view['rgb'] = dict(self.rgb.metadata, available=self.rgb.jpeg is not None,
                                    fresh=self.rgb.snapshot() is not None)

    def destroy_node(self):
        self.http.shutdown(); self.http.server_close(); self.thread.join(timeout=2)
        return super().destroy_node()


def handler(node):
    page = files('deux_task_gui').joinpath('index.html').read_bytes()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args): pass  # Do not log credentials or control input.

        def reply(self, code, body, mime='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', mime); self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control','no-store'); self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('X-Frame-Options','DENY'); self.send_header('Referrer-Policy','no-referrer')
            self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass

        def authorized(self):
            return secrets.compare_digest(self.headers.get('Authorization','').encode(), ('Bearer '+node.token).encode())

        def do_GET(self):
            if self.path == '/': self.reply(200, page, 'text/html; charset=utf-8')
            elif self.path == '/api/frame':
                if not self.authorized(): self.reply(401, {'error':'인증 토큰을 확인하세요'}); return
                frame = node.frame_snapshot()
                self.reply(200,frame,'image/jpeg') if frame else self.reply(503,{'error':'SIM 영상 대기 중'})
            elif self.path == '/api/rgb':
                if not self.authorized(): self.reply(401, {'error':'인증 토큰을 확인하세요'}); return
                frame = node.rgb.snapshot()
                self.reply(200,frame,'image/jpeg') if frame else self.reply(503,{'error':'ZED RGB 영상 대기 중'})
            elif self.path == '/api/tracking-roi':
                if not self.authorized(): self.reply(401, {'error':'인증 토큰을 확인하세요'}); return
                try:
                    from deux_task_perception.tracking_roi import read_roi
                    self.reply(200,{'roi':read_roi(node.get_parameter('tracking_roi_file').value)})
                except (ValueError,OSError) as exc:self.reply(400,{'error':str(exc)})
            elif self.path == '/api/state':
                self.reply(200, node.snapshot()) if self.authorized() else self.reply(401, {'error':'인증 토큰을 확인하세요'})
            else: self.reply(404, {'error':'not found'})

        def do_POST(self):
            if self.path not in ('/api/command','/api/tracking-roi'): self.reply(404, {'error':'not found'}); return
            if not self.authorized(): self.reply(401, {'error':'인증 토큰을 확인하세요'}); return
            origin = self.headers.get('Origin')
            if origin and urlsplit(origin).netloc != self.headers.get('Host'):
                self.reply(403, {'error':'origin mismatch'}); return
            try:
                size = int(self.headers.get('Content-Length','0'))
                if not 0 < size <= 2048: raise ValueError('invalid body size')
                data = json.loads(self.rfile.read(size))
                if self.path=='/api/tracking-roi':
                    view=node.snapshot()
                    if not view.get('connected') or view.get('status',{}).get('active') or view.get('pending') or view.get('ready_pending'):
                        self.reply(409,{'error':'작업 중에는 ROI를 변경할 수 없습니다. 중지 후 저장하세요.'});return
                    from deux_task_perception.tracking_roi import write_roi
                    saved=write_roi(node.get_parameter('tracking_roi_file').value,data)
                    self.reply(200,{'roi':saved});return
                if not isinstance(data,dict) or set(data)-{'command','target_id'}:
                    raise ValueError('unsupported command fields')
                if node.perception_only():self.reply(409,{'error':'인식 전용 테스트에서는 로봇 명령을 실행하지 않습니다'});return
                cmd, target = data['command'], data.get('target_id','')
                if cmd not in ('start','stop','restart','ready','motion_approve','motion_reject') or not isinstance(target,str) or len(target)>128:
                    raise ValueError('invalid command or target')
                node.commands.put_nowait((cmd,target.strip()))
                self.reply(202, {'accepted':True})
            except queue.Full: self.reply(409, {'error':'요청 처리 중입니다'})
            except (ValueError, KeyError, TypeError): self.reply(400, {'error':'invalid request'})
    return Handler


def main(args=None):
    rclpy.init(args=args); node = TaskGui()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
