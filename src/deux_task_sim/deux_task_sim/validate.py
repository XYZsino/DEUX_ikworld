"""Run real ROS task/planner/execution/bridges against MuJoCo, with a bounded audit."""
import os
os.environ.setdefault('MUJOCO_GL','egl')
os.environ.setdefault('OPENBLAS_NUM_THREADS','1')
import argparse,json,time,traceback
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from deux_task_msgs.action import RunTask
from deux_task_runtime.task_node import TaskNode
from deux_task_runtime.motion_planner_node import MotionPlannerNode
from deux_task_runtime.execution_node import ExecutionNode
from deux_task_adapters.arm_bridge_node import ArmBridgeNode
from deux_task_adapters.hand_bridge_node import HandBridgeNode
from deux_task_perception.state_observation_node import StateObservationNode
from deux_task_perception.barcode_bridge_node import BarcodeBridgeNode
from .world import load_sim,configure
from .node import MujocoNode


def params(**kwargs):return [Parameter(k,value=v) for k,v in kwargs.items()]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-repository',required=True);parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--seed',type=int,default=4);parser.add_argument('--wall-timeout',type=float,default=300.)
    parser.add_argument('--case',choices=['transfer','no_barcode','observation_loss','cancel'],default='transfer')
    parser.add_argument('--record',action='store_true')
    parser.add_argument('--observe-flow',action='store_true',help='Record actual flow; task failure is a finding, not a validation-process failure')
    a=parser.parse_args();a.output=a.output.resolve();a.output.mkdir(parents=True,exist_ok=True)
    sim=load_sim(a.sim_repository,a.output,a.seed)
    config,frame,grasp=configure(sim,a.output)
    (a.output/'grasp.json').write_text(json.dumps(grasp['record'],indent=2))
    rclpy.init();ex=SingleThreadedExecutor();nodes=[];result=None;error=None;handle=None;action_status=None
    driver=MujocoNode(sim,config,a.output,reader_enabled=a.case!='no_barcode',dropout=a.case=='observation_loss',record=a.record)
    from .trace import TraceRecorder
    recorder=TraceRecorder(driver,lambda:sim.data.time)
    common=dict(config_file=str(a.output/'installation.yaml'),use_sim_time=True)
    try:
        nodes=[driver,
            StateObservationNode(parameter_overrides=params(use_sim_time=True,left_ee_frame='left_grasp',right_ee_frame='right_grasp')),
            MotionPlannerNode(parameter_overrides=params(**common)),
            ExecutionNode(parameter_overrides=params(**common,dry_run=False)),
            ArmBridgeNode(parameter_overrides=params(**common,hardware_enabled=True)),
            HandBridgeNode(parameter_overrides=params(**common,hardware_enabled=True,mapping_file=str(a.output/'hand_mapping.yaml'))),
            BarcodeBridgeNode(parameter_overrides=params(use_sim_time=True,simulated=True)),
            TaskNode(parameter_overrides=params(**common))]
        for node in nodes:ex.add_node(node)
        task=nodes[-1];client=ActionClient(driver,RunTask,'/task/run')
        for observed,name,src,dst in [
            (client,'/task/run','scenario_client','task_node'),
            (task.planner,'/task/plan_motion','task_node','motion_planner_node'),
            (task.executor_client,'/task/execute_motion','task_node','execution_node'),
            (nodes[3].bridge_clients[0],'/task/arm_motion','execution_node','arm_bridge_node'),
            (nodes[3].bridge_clients[1],'/task/hand_motion','execution_node','hand_bridge_node'),
            (nodes[4].client,'/arm/deux_arm_controller/follow_joint_trajectory','arm_bridge_node','mujoco_driver_node')]:
            recorder.watch(observed,name,src,dst)
        end=time.monotonic()+a.wall_timeout
        def spin(predicate):
            while not predicate():
                if time.monotonic()>end:raise TimeoutError('simulation wall deadline')
                ex.spin_once(timeout_sec=.01)
        def ready():
            if not client.server_is_ready() or not task.planner.server_is_ready() or not task.executor_client.server_is_ready():return False
            try:task.measured();task.target('bread_3');return True
            except ValueError:return False
        spin(ready)
        request=client.send_goal_async(RunTask.Goal(target_id='bread_3'));spin(request.done);handle=request.result()
        if not handle.accepted:raise RuntimeError('task rejected')
        future=handle.get_result_async();canceled=False
        while not future.done():
            if time.monotonic()>end:raise TimeoutError('simulation wall deadline')
            ex.spin_once(timeout_sec=.01)
            if a.case=='cancel' and driver.stage=='Approach' and not canceled:
                handle.cancel_goal_async();canceled=True
        action_status=future.result().status
        result=future.result().result
        identity=bytes(handle.goal_id.uuid).hex()
        if not any(e['name']=='/task/run' and e['phase']=='result' and e['goal_id']==identity for e in recorder.events):
            from .trace import summary
            payload=summary(result);payload['action_status']=action_status
            recorder.add('action','/task/run','task_node',['scenario_client'],'result',payload,identity)
        # Continue physics after terminal task result to observe settling, not just command completion.
        settle=sim.data.time+1.
        spin(lambda:sim.data.time>=settle)
    except Exception as exc:
        error=f'{type(exc).__name__}: {exc}';traceback.print_exc()
        if handle and handle.accepted:
            cancel=handle.cancel_goal_async()
            for _ in range(100):ex.spin_once(timeout_sec=.01)
    finally:
        from transfer_sim_adapter import transform
        from reader_pick import scan_metrics
        base=np.linalg.inv(transform(sim.data,'base_link'));obj=base@transform(sim.data,'bread_3')
        initial=np.array(driver.records[0]['object_pose']) if driver.records else obj
        lift=max((r['object_pose'][2][3]-initial[2,3] for r in driver.records),default=0.)
        bounds=config['destination_bounds'];inside=bool(np.all(obj[:3,3]>=bounds['min']) and np.all(obj[:3,3]<=bounds['max']))
        report=dict(case=a.case,seed=a.seed,action_status=action_status,success=bool(result and result.success),reason=result.reason if result else error,
            error=error,barcode=result.barcode if result else '',physical_lift_m=lift,physical_in_destination=inside,
            simulated_reader_events=driver.reader_events,physics_warnings=sim.data.warning.number.tolist(),
            max_active_welds=driver.max_welds,grasp_assist=sim.grasp_assist,conveyor_speed=sim.conveyor.speed,
            sensor_mode='visibility-gated simulation truth 6D + analytic label surface; not YOLO/ICP',
            stages=driver.stage_events,frames=len(driver.records),simulation_time_s=float(sim.data.time))
        report['verified_transfer']=bool(report['success'] and lift>=config['lift_height']*.5 and inside and driver.reader_events and not any(report['physics_warnings']) and driver.max_welds==0)
        reason=report['reason'] or ''
        report['expected_fault_observed'] = bool(not report['success'] and not error and (
            (a.case=='cancel' and action_status==5) or
            (a.case=='no_barcode' and 'barcode missing' in reason) or
            (a.case=='observation_loss' and any(word in reason for word in ('object stale','objects unavailable','target_observation','required object pose')))))
        (a.output/'result.json').write_text(json.dumps(report,indent=2));(a.output/'trace.json').write_text(json.dumps(driver.records))
        (a.output/'ros-events.json').write_text(json.dumps(dict(topics=recorder.topics,events=recorder.events),ensure_ascii=False))
        (a.output/'video-frames.json').write_text(json.dumps(driver.video_frames))
        print(json.dumps(report,indent=2),flush=True)
        driver.timer.cancel()
        driver.render_recording()
        ex.shutdown()
        for node in reversed(nodes):node.destroy_node()
        if driver not in nodes:driver.destroy_node()
        rclpy.shutdown()
    if a.observe_flow:
        if report['error'] or not recorder.events:raise SystemExit(1)
        return
    if a.case=='transfer' and not report['verified_transfer']:raise SystemExit(1)
    if a.case!='transfer' and not report['expected_fault_observed']:raise SystemExit(1)

if __name__=='__main__':main()
