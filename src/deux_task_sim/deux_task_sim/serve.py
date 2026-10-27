"""Interactive simulation: expose the same RunTask action used on real hardware."""
import argparse
import os
from pathlib import Path
os.environ.setdefault('MUJOCO_GL', 'egl')
import rclpy
from rclpy.executors import SingleThreadedExecutor
from .validate import params
from .world import load_sim, configure, initialize_urdf_zero
from .node import MujocoNode
from deux_task_runtime.task_node import TaskNode
from deux_task_runtime.motion_planner_node import MotionPlannerNode
from deux_task_runtime.execution_node import ExecutionNode
from deux_task_perception.state_observation_node import StateObservationNode
from deux_task_perception.barcode_node import BarcodeNode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sim-repository', required=True)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--conveyor-speed',type=float,default=-.08)
    parser.add_argument('--seed', type=int, default=4)
    parser.add_argument('--perception',choices=['head-rgbd','legacy-truth'],default='head-rgbd')
    parser.add_argument('--weights',type=Path,help='Trained bag/barcode segmentation checkpoint for head-rgbd')
    parser.add_argument('--device',default='cuda:0')
    parser.add_argument('--ik-backend',choices=['scipy','pyroki','curobo'],default=None,
                        help='Override head-rgbd installation IK engine (default: YAML)')
    parser.add_argument('--gui',action='store_true',help='Serve loopback browser controls in this ROS executor')
    parser.add_argument('--gui-port',type=int,default=8080)
    parser.add_argument('--gui-host',default='127.0.0.1')
    a = parser.parse_args()
    if a.perception!='head-rgbd' and a.ik_backend is not None:
        parser.error('--ik-backend requires head-rgbd perception')
    if a.perception=='head-rgbd' and (a.weights is None or not a.weights.is_file()):
        parser.error('--weights must name a trained bag/barcode segmentation checkpoint')
    a.output = a.output.resolve(); a.output.mkdir(parents=True, exist_ok=True)
    sim = load_sim(a.sim_repository, a.output, a.seed,conveyor_speed=a.conveyor_speed if a.perception=='head-rgbd' else 0.)
    if a.perception=='head-rgbd':
        from .head_installation import configure_head
        config=configure_head(sim,a.output,ik_backend=a.ik_backend)
    else:config,_,_=configure(sim,a.output)
    initialize_urdf_zero(sim)
    rclpy.init(); ex = SingleThreadedExecutor(); nodes = []
    common = dict(config_file=str(a.output/'installation.yaml'), use_sim_time=True)
    try:
        if a.perception=='head-rgbd':
            from .head_stack import build_head_stack
            nodes=build_head_stack(sim,config,a.output,a.weights,device=a.device)
        else:
            nodes = [MujocoNode(sim, config, a.output),
                StateObservationNode(parameter_overrides=params(use_sim_time=True, left_ee_frame='left_grasp', right_ee_frame='right_grasp')),
                MotionPlannerNode(parameter_overrides=params(**common)),
                ExecutionNode(parameter_overrides=params(**common, dry_run=False, hardware_enabled=True, mapping_file=str(a.output/'hand_mapping.yaml'))),
                BarcodeNode(parameter_overrides=params(use_sim_time=True, simulated=True)),
                TaskNode(parameter_overrides=params(**common))]
        # Interactive sessions keep bounded diagnostics; no continuous video buffer.
        from collections import deque
        nodes[0].records = deque(maxlen=300)
        nodes[0].stage_qpos = deque(maxlen=30)
        nodes[0].stage_events = deque(maxlen=100)
        nodes[0].reader_events = deque(maxlen=100)
        if a.gui:
            # Keep the staged source available while the operator opens/authenticates
            # the browser. Once the first goal is admitted the belt runs continuously.
            # This is SIM startup behaviour, never a reset/reposition during a task.
            task = nodes[-1]
            sim.conveyor.set_speed(0.)
            from deux_task_gui.node import TaskGui
            gui=TaskGui(parameter_overrides=params(deployment='sim',port=a.gui_port,host=a.gui_host))
            nodes.append(gui)
            from .web_view import SimWebView
            web_view=SimWebView(sim,gui)
            gui.create_timer(.33,web_view.render)
            def start_conveyor():
                if task.action.busy:
                    sim.conveyor.set_speed(a.conveyor_speed if a.perception=='head-rgbd' else 0.)
                    gui.destroy_timer(belt_timer)
            belt_timer=gui.create_timer(.01,start_conveyor)
        for node in nodes: ex.add_node(node)
        nodes[0].get_logger().info('Interactive sim ready; waiting for /task/run (no automatic goal)')
        ex.spin()
    except KeyboardInterrupt: pass
    finally:
        ex.shutdown()
        for node in reversed(nodes): node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
