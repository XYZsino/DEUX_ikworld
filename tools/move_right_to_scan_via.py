"""One-shot measured ready -> stored scan VIA; left/fingers stay untouched.
Plan only unless --execute. Uses the existing conveyor-only geometry policy.
"""
import argparse,json,time
import numpy as np
import rclpy
from scipy.interpolate import CubicHermiteSpline
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from prepare_grid_ready import Preparation
from deux_task_core.model import ARM_NAMES
from deux_task_runtime.common import ros_time
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_bt_support import time_prepare_path
from lookup_lift_scan import VIA
from lookup_fast_scan import ROOT


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true');args=parser.parse_args()
    rclpy.init();node=Preparation();handle=pending=None
    out=ROOT/'.setup/right-scan-via';out.mkdir(parents=True,exist_ok=True)
    report=dict(execution_requested=args.execute,left_arm_commanded=False,hand_commands=False)
    try:
        until=time.monotonic()+8
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.02)
            if node.arm is not None and node.robot_status is not None and time.monotonic()>until-5:break
        names={n for n,_ in node.get_node_names_and_namespaces()}
        if names & {'task_node','grid_grasp_trial','execution_node'}:raise RuntimeError('stop lookup BT before this motion')
        library=ClassifiedGridLibrary(node.config,directory=ROOT/'artifacts/right_grasp_grid_classified',recommended=False,yaw_secondary=True,load_rotation_paths=False)
        for _ in range(30):rclpy.spin_once(node,timeout_sec=.005)
        actual=node.actual();left=actual[:7].copy();node.guard(left)
        ready=np.r_[node.config['home_joints'][:7],library.ready[14:21]]
        report['ready_error_rad']=float(np.max(abs(actual-ready)))
        if report['ready_error_rad']>.05:raise RuntimeError('arms are not at ready; no motion sent')
        previous=actual.copy();began=time.monotonic()
        while time.monotonic()-began<.4:
            rclpy.spin_once(node,timeout_sec=.01)
            if np.max(abs(node.guard(left)-previous))>.005:raise RuntimeError('arms still moving; no motion sent')
        start=library.ready.copy();start[:7]=left;start[14:21]=actual[7:];goal=start.copy();goal[14:21]=VIA
        plan=time_prepare_path(node.robot,[start,goal],lambda:False,.15,.3)
        times=plan['times'];path=plan['path'];curve=CubicHermiteSpline(times,path,np.zeros_like(path))
        report.update(start_arm_rad=actual.tolist(),target_right_rad=VIA.tolist(),duration_s=float(times[-1]),minimum_conveyor_clearance_m=plan['minimum_clearance'])
        (out/'plan.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True)
        if not args.execute:return
        for _ in range(30):rclpy.spin_once(node,timeout_sec=.005)
        if np.max(abs(node.guard(left)-actual))>.02:raise RuntimeError('start moved during planning')
        if not node.client.wait_for_server(timeout_sec=3):raise RuntimeError('arm controller unavailable')
        trajectory=JointTrajectory(joint_names=list(ARM_NAMES[7:]))
        start_at=node.now()+.75;trajectory.header.stamp=ros_time(start_at)
        trajectory.points=[JointTrajectoryPoint(positions=q[14:21].tolist(),velocities=[0.]*7,time_from_start=ros_time(float(t),Duration)) for t,q in zip(times,path)]
        pending=node.client.send_goal_async(FollowJointTrajectory.Goal(trajectory=trajectory))
        handle=node.wait(pending,.5,lambda:node.guard(left))
        if not handle.accepted:raise RuntimeError('controller rejected via motion')
        if node.now()>=start_at:raise RuntimeError('late acceptance')
        future=handle.get_result_async();deadline=time.monotonic()+times[-1]+10
        while not future.done():
            rclpy.spin_once(node,timeout_sec=.01);measured=node.guard(left)
            expected=curve(np.clip(node.now()-start_at,0,times[-1]))[14:21]
            if np.max(abs(measured[7:]-expected))>node.config['tracking_tolerance']:raise RuntimeError('tracking error')
            if time.monotonic()>deadline:raise TimeoutError('via motion overdue')
        result=future.result()
        if result.status!=4 or result.result.error_code:raise RuntimeError('controller failed: '+result.result.error_string)
        final=node.guard(left);error=float(np.max(abs(final[7:]-VIA)))
        report.update(controller_succeeded=True,final_max_error_rad=error,left_change_rad=float(np.max(abs(final[:7]-left))),final_arm_rad=final.tolist())
        if error>.03:raise RuntimeError('via final error > .03rad')
        handle=None;report.update(success=True,final_max_error_rad=error,left_change_rad=float(np.max(abs(final[:7]-left))),final_arm_rad=final.tolist())
        print(json.dumps(report),flush=True)
    except BaseException as e:
        report.update(success=False,reason=str(e));raise
    finally:
        if handle is not None and handle.accepted:
            try:node.wait(handle.cancel_goal_async(),2,lambda:None)
            except Exception:pass
        elif pending is not None and not report.get('success'):
            def cancel_late(f):
                h=f.result()
                if h.accepted:h.cancel_goal_async()
            pending.add_done_callback(cancel_late)
            for _ in range(50):rclpy.spin_once(node,timeout_sec=.02)
        (out/'result.json').write_text(json.dumps(report,indent=2))
        node.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
