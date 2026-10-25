"""Read-only topic observer and ActionClient boundary instrumentation.

Topic events are actual DDS subscriber receipts; action events are observed API
send/ack/feedback/result/cancel boundaries, not a packet capture.
"""
import time
from uuid import uuid4
from rclpy.qos import qos_profile_sensor_data
from unique_identifier_msgs.msg import UUID
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from deux_hand_msgs.msg import HandCtlMessage
from deux_task_msgs.msg import StateObservation,ObjectDetectionArray,BarcodeTarget,PerceptionRequest,ScanSession,BarcodeRead,TaskStatus
from deux_task_runtime.common import seconds,LATCHED

STATE_CONSUMERS=['task_node','motion_planner_node','execution_node']
TOPICS=[
 ('/arm/joint_states',JointState,'mujoco_driver_node',['state_observation_node','arm_bridge_node'],False),
 ('/sim/hand/left/joint_states',JointState,'mujoco_driver_node',['hand_bridge_node'],False),
 ('/sim/hand/right/joint_states',JointState,'mujoco_driver_node',['hand_bridge_node'],False),
 ('/task/hand_state',JointState,'hand_bridge_node',['state_observation_node'],False),
 ('/task/object_detections',ObjectDetectionArray,'mujoco_driver_node',['state_observation_node'],False),
 ('/task/state_observation',StateObservation,'state_observation_node',STATE_CONSUMERS,False),
 ('/task/perception_request',PerceptionRequest,'task_node',['mujoco_driver_node'],True),
 ('/task/barcode_target',BarcodeTarget,'mujoco_driver_node',['task_node','execution_node'],False),
 ('/task/scan_session',ScanSession,'task_node',['barcode_bridge_node','mujoco_driver_node'],True),
 ('/barcode/raw',String,'mujoco_driver_node',['barcode_bridge_node'],False),
 ('/task/barcode_read',BarcodeRead,'barcode_bridge_node',['task_node'],False),
 ('/task/status',TaskStatus,'task_node',[],True),
 ('/hand/left/cmd',HandCtlMessage,'hand_bridge_node',['mujoco_driver_node'],False),
 ('/hand/right/cmd',HandCtlMessage,'hand_bridge_node',['mujoco_driver_node'],False),
]


def summary(msg):
    out={}
    for key in ('stage','state','mode','target_id','track_id','request_id','session_id','plan_id','reason','success','value','active','simulated','progress','status','sequence','joints_valid','objects_valid','left_ee_valid','right_ee_valid','full_orientation'):
        if hasattr(msg,key):out[key]=getattr(msg,key)
    if hasattr(msg,'header'):out['acquired_s']=seconds(msg.header.stamp)
    if hasattr(msg,'data'):out['data']=str(msg.data)[:160]
    if hasattr(msg,'position'):out['joint_count']=len(msg.position)
    if hasattr(msg,'detections'):out['tracks']=[d.track_id for d in msg.detections]
    if hasattr(msg,'motors'):out['motor_count']=len(msg.motors)
    if hasattr(msg,'plan'):
        out.update(summary(msg.plan))
    if hasattr(msg,'trajectory'):
        out['trajectory_points']=len(msg.trajectory.points)
        if msg.trajectory.points:out['duration_s']=seconds(msg.trajectory.points[-1].time_from_start)
    return out


class TraceRecorder:
    def __init__(self,node,simulation_time):
        self.node=node;self.now=simulation_time;self.events=[];self.subs=[];self.started=time.monotonic()
        self.topics=[dict(name=name,type=typ.__module__.split('.')[0]+'/msg/'+typ.__name__,source=src,targets=targets) for name,typ,src,targets,_ in TOPICS]
        for name,typ,src,targets,latched in TOPICS:
            self.subs.append(node.create_subscription(typ,name,
                lambda m,n=name,s=src,t=targets:self.add('topic',n,s,t,'received',summary(m)),
                LATCHED if latched else qos_profile_sensor_data))

    def add(self,kind,name,src,targets,phase,payload,goal_id=''):
        self.events.append(dict(id=len(self.events),time_s=float(self.now()),wall_s=time.monotonic()-self.started,
            kind=kind,name=name,source=src,targets=targets,phase=phase,payload=payload,goal_id=goal_id))

    def watch(self,client,name,source,target):
        original=client.send_goal_async
        def send(goal,feedback_callback=None,goal_uuid=None):
            uid=goal_uuid or UUID(uuid=list(uuid4().bytes));identity=bytes(uid.uuid).hex()
            self.add('action',name,source,[target],'goal_sent',summary(goal),identity)
            def feedback(msg):
                self.add('action',name,target,[source],'feedback',summary(msg.feedback),identity)
                if feedback_callback:feedback_callback(msg)
            future=original(goal,feedback_callback=feedback,goal_uuid=uid)
            def accepted(f):
                try:
                    handle=f.result()
                    self.add('action',name,target,[source],'accepted' if handle.accepted else 'rejected',{},identity)
                    if not handle.accepted:return
                    get_result=handle.get_result_async
                    reported=[False]
                    def observe_result():
                        result=get_result()
                        def finished(r):
                            if reported[0]:return
                            reported[0]=True
                            value=r.result();payload=summary(value.result);payload['action_status']=value.status
                            self.add('action',name,target,[source],'result',payload,identity)
                        result.add_done_callback(finished)
                        return result
                    handle.get_result_async=observe_result
                    cancel=handle.cancel_goal_async
                    def cancel_goal():
                        self.add('action',name,source,[target],'cancel_sent',{},identity)
                        canceled=cancel()
                        canceled.add_done_callback(lambda f:self.add('action',name,target,[source],'cancel_ack',
                            {'canceling_count':len(f.result().goals_canceling)},identity))
                        return canceled
                    handle.cancel_goal_async=cancel_goal
                except Exception as error:
                    self.add('observer',name,source,[],'observer_error',{'reason':repr(error)},identity)
            future.add_done_callback(accepted)
            return future
        client.send_goal_async=send
