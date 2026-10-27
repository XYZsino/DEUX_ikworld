"""Actual ROS image→YOLO/ByteTrack→selected ICP→PickScan scenario recorder."""
import argparse
import hashlib
import json
import os
import time
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import rclpy
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_client_names_and_types_by_node
from rclpy.executors import SingleThreadedExecutor
from deux_task_msgs.action import RunTask
from deux_task_msgs.msg import Observation,DetectionArray,DetectionFrame,VisualEvidence,VisualEvidenceArray
from deux_task_runtime.head_task_node import HeadTaskNode
from deux_task_core.ik_backends import backend_name
from .validate import params,StateObservationNode,MotionPlannerNode,ExecutionNode,BarcodeNode
from .world import load_sim,initialize_urdf_zero
from .head_installation import configure_head
from .node import MujocoNode
from .trace import TraceRecorder,TOPICS


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sim-repository',required=True);p.add_argument('--output',required=True,type=Path)
    p.add_argument('--weights',required=True,type=Path);p.add_argument('--record',action='store_true')
    p.add_argument('--conveyor-speed',type=float,default=-.08)
    p.add_argument('--ik-backend',choices=['scipy','pyroki','curobo'],default=None)
    p.add_argument('--wall-timeout',type=float,default=300.)
    a=p.parse_args();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=True)
    sim=load_sim(a.sim_repository,a.output,conveyor_speed=a.conveyor_speed)
    config=configure_head(sim,a.output,ik_backend=a.ik_backend);initialize_urdf_zero(sim)
    from deux_conveyor import BREAD_ASSET,BREAD_MODEL
    rclpy.init();ex=SingleThreadedExecutor();nodes=[];result=None;failure=None;handle=None
    common=dict(config_file=str(a.output/'installation.yaml'),use_sim_time=True)
    try:
        from .head_stack import build_head_stack
        nodes=build_head_stack(sim,config,a.output,a.weights,record=a.record)
        driver,perception=nodes[:2]
        for n in nodes:ex.add_node(n)
        task=nodes[-1]
        topics=[]
        for name,typ,src,dst,latched in TOPICS:
            if name=='/task/object_detections':src='perception_node'
            if name=='/task/barcode_target':src='perception_node'
            if name=='/task/perception_request':dst=['perception_node']
            topics.append((name,typ,src,dst,latched))
        topics += [('/task/observation',Observation,'mujoco_driver_node',['perception_node'],False),
            ('/task/detections',DetectionArray,'perception_node',['task_node'],False),
            ('/task/visual_candidates',VisualEvidenceArray,'perception_node',['task_node'],False),
            ('/task/visual_evidence',VisualEvidence,'perception_node',['task_node'],False)]
        recorder=TraceRecorder(driver,lambda:sim.data.time,topics)
        client=ActionClient(driver,RunTask,'/task/run')
        for c,name,src,dst in [(client,'/task/run','scenario_client','task_node'),
            (task.planner,'/task/plan_motion','task_node','motion_planner_node'),
            (task.executor_client,'/task/execute_motion','task_node','execution_node'),
            (next(n for n in nodes if n.get_name()=='execution_node').arm.client,'/arm/deux_arm_controller/follow_joint_trajectory','execution_node','mujoco_driver_node')]:recorder.watch(c,name,src,dst)
        deadline=time.monotonic()+a.wall_timeout
        def spin(predicate):
            while not predicate():
                if time.monotonic()>deadline:raise TimeoutError('scenario wall deadline')
                ex.spin_once(timeout_sec=.01)
        def ready():
            try:task.robot_ready()
            except ValueError:return False
            return client.server_is_ready()
        spin(ready)
        future=client.send_goal_async(RunTask.Goal());spin(future.done);handle=future.result()
        if not handle.accepted:raise RuntimeError('task rejected')
        future=handle.get_result_async();spin(future.done);response=future.result()
        result=dict(success=response.result.success,reason=response.result.reason,barcode=response.result.barcode,action_status=response.status)
        until=sim.data.time+1.;spin(lambda:sim.data.time>=until)
    except Exception as exc:
        import traceback
        traceback.print_exc();failure=f'{type(exc).__name__}: {exc}'
        if handle and handle.accepted:
            handle.cancel_goal_async()
            end=time.monotonic()+5.
            while time.monotonic()<end:ex.spin_once(timeout_sec=.01)
    finally:
        if nodes:
            driver.timer.cancel()
            data=dict(result=result,error=failure,simulation_time_s=sim.data.time,
                ik_backend=backend_name(config),
                conveyor_speed_m_s=a.conveyor_speed,filter_rejections=perception.estimator.filter_rejections,
                stages=driver.stage_events,physics_warnings=sim.data.warning.number.tolist(),
                max_environment_penetration_m=driver.max_environment_penetration_m,
                max_active_welds=driver.max_welds,
                reader_model='printed barcode UV center; optical occlusion ray; fixture EAN, not image decoding',
                max_fixture_penetration_m=driver.max_fixture_penetration_m,
                max_object_penetration_m=driver.max_object_penetration_m,
                object_pose_publishers=[x.node_name for x in driver.get_publishers_info_by_topic('/task/object_detections')],
                runtime_nodes=[n.get_name() for n in nodes],
                action_clients=[name for n in nodes for name, _ in get_action_client_names_and_types_by_node(n, n.get_name(), n.get_namespace())],
                sensor_mode='rendered ZED 2i RGB-D; learned YOLO11n + ByteTrack; selected-target ICP',
                camera_mount=json.loads((a.output/'camera-mount.json').read_text()),
                weights_sha256=hashlib.sha256(a.weights.read_bytes()).hexdigest(),
                perception=dict(detection_published=perception.accepted,detection_rejected=perception.rejected,
                    detection_status=perception.status,icp_published=perception.icp_accepted,icp_rejected=perception.icp_rejected,icp_status=perception.status))
            (a.output/'result.json').write_text(json.dumps(data,indent=2))
            (a.output/'ros-events.json').write_text(json.dumps(dict(topics=recorder.topics,events=recorder.events)))
            (a.output/'trace.json').write_text(json.dumps(driver.records))
            (a.output/'video-frames.json').write_text(json.dumps(driver.video_frames))
            driver.render_recording()
        ex.shutdown()
        for n in reversed(nodes):n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    if failure:raise SystemExit(1)

if __name__=='__main__':main()
