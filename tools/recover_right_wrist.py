"""Explicit, bounded inward recovery of arm_r_joint6; no general limit override.

Default is read-only planning. --execute sends this recovery only, not the BT.
Other joints hold position. Hand commands and installed limits are unchanged.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np


def validate_recovery(t, path, limits, velocity, acceleration, index):
    t, path = np.asarray(t), np.asarray(path)
    if (path.shape != (len(t), 14) or len(t) < 3 or not np.isfinite(path).all()
            or not np.isfinite(t).all() or t[0] != 0 or np.any(np.diff(t) <= 0)):
        raise ValueError('malformed recovery')
    lo, hi = limits[:, 0], limits[:, 1]
    excess = np.maximum(np.maximum(lo-path[0], path[0]-hi), 0)
    if not (0 < excess[index] <= np.deg2rad(3)) or np.any(np.delete(excess, index) > 0):
        raise ValueError('recovery only permits this wrist up to 3 degrees outside its limit')
    others = [i for i in range(14) if i != index]
    if not np.allclose(path[:, others], path[0, others], atol=1e-10, rtol=0):
        raise ValueError('other joints must hold')
    direction = -1 if path[0, index] > hi[index] else 1
    if np.any(direction*np.diff(path[:, index]) < -1e-10):
        raise ValueError('outward recovery step')
    if not (lo[index]+np.deg2rad(4.9) <= path[-1,index] <= hi[index]-np.deg2rad(4.9)):
        raise ValueError('recovery endpoint requires interior margin')
    if np.any(path[-1] < lo) or np.any(path[-1] > hi):
        raise ValueError('endpoint outside limits')
    dt = np.diff(t)
    v = np.diff(path, axis=0)/dt[:, None]
    a = np.diff(np.vstack([np.zeros(14), v, np.zeros(14)]), axis=0)/np.r_[dt, dt[-1]][:,None]
    if np.any(abs(v) > np.minimum(velocity, np.deg2rad(4))*1.001):
        raise ValueError('recovery velocity limit')
    if np.any(abs(a) > np.minimum(acceleration, np.deg2rad(4))*1.001):
        raise ValueError('recovery acceleration limit')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from deux_task_msgs.msg import TaskStatus
    from deux_task_core.model import RobotModel, JOINT_NAMES, ARM_NAMES
    from deux_task_runtime.common import RuntimeNode, LATCHED, current, joint_array, ros_time
    root=Path('/home/xyz_thor/DEUX_ikworld')
    rclpy.init()
    node=RuntimeNode('bounded_wrist_recovery', observe_state=True, observe_status=True,
        parameter_overrides=[Parameter('config_file', value=str(root/'src/deux_task_bringup/config/installation.yaml'))])
    state={}
    subs=[node.create_subscription(JointState, '/arm/joint_states', lambda m:state.update(arm=m), qos_profile_sensor_data),
          node.create_subscription(TaskStatus, '/task/status', lambda m:state.update(task=m), LATCHED)]
    client=ActionClient(node, FollowJointTrajectory, '/arm/deux_arm_controller/follow_joint_trajectory')
    robot=RobotModel(node.config['robot'])
    indices=[JOINT_NAMES.index(n) for n in ARM_NAMES]
    limits=robot.limits[indices]; index=list(ARM_NAMES).index('arm_r_joint6')
    def spin(): rclpy.spin_once(node, timeout_sec=.02)
    def measured():
        m=state.get('arm')
        if m is None or not current(m.header.stamp,node.now(),node.config['max_joint_age']):
            raise ValueError('arm feedback missing/stale')
        return joint_array(m,ARM_NAMES)
    def gate():
        node.hardware_gate(); node.robot_ready()
        m=state.get('task')
        if m is None or not current(m.header.stamp,node.now(),1.) or m.active:
            raise ValueError('BT must be inactive with fresh status')
    handle=None
    report={}
    try:
        end=time.monotonic()+2
        while time.monotonic()<end: spin()
        gate(); start=measured(); target=start.copy()
        target[index]=limits[index,1]-np.deg2rad(5) if start[index]>limits[index,1] else limits[index,0]+np.deg2rad(5)
        t=np.linspace(0,4,201); u=t/4; path=start+(3*u*u-2*u*u*u)[:,None]*(target-start)
        validate_recovery(t,path,limits,robot.velocity[indices],robot.acceleration[indices],index)
        report.update(start_deg=np.rad2deg(start).tolist(),target_deg=np.rad2deg(target).tolist(),duration_s=4,validated=True)
        print(json.dumps(report),flush=True)
        if not args.execute:return
        if not client.wait_for_server(timeout_sec=2):raise ValueError('driver unavailable')
        gate()
        if np.max(abs(measured()-start))>np.deg2rad(.2):raise ValueError('start changed; replan')
        tr=JointTrajectory(joint_names=list(ARM_NAMES))
        start_at=node.now()+.75; tr.header.stamp=ros_time(start_at)
        for ts,row in zip(t,path):
            tr.points.append(JointTrajectoryPoint(positions=row.tolist(), time_from_start=ros_time(ts,Duration)))
        future=client.send_goal_async(FollowJointTrajectory.Goal(trajectory=tr))
        # Keep ownership of a late acceptance so it can still be canceled.
        while not future.done():spin()
        handle=future.result()
        if not handle.accepted:raise ValueError('driver rejected recovery')
        if node.now()>=start_at:raise ValueError('acceptance missed scheduled start')
        result=handle.get_result_async(); previous=node.now()
        initial_excess=np.maximum(np.maximum(limits[:,0]-start,start-limits[:,1]),0)
        best_excess=initial_excess[index]
        while True:
            spin(); gate(); now=node.now()
            if now<previous:raise ValueError('clock moved backwards')
            previous=now; actual=measured()
            excess=np.maximum(np.maximum(limits[:,0]-actual,actual-limits[:,1]),0)
            if np.any(np.delete(excess,index)>robot.joint_limit_tolerance):raise ValueError('another joint left limits')
            if excess[index]>best_excess+np.deg2rad(.3):raise ValueError('wrist moving farther outside limit')
            best_excess=min(best_excess,excess[index])
            expected=np.array([np.interp(max(0,now-start_at),t,path[:,i]) for i in range(14)])
            if np.max(abs(actual-expected))>np.deg2rad(3):raise ValueError('recovery tracking error')
            if result.done():
                value=result.result()
                if value.status!=4 or value.result.error_code!=0:raise ValueError('driver failed: '+value.result.error_string)
                if np.any(excess>robot.joint_limit_tolerance):raise ValueError('recovery did not reach joint limits interior')
                if np.max(abs(actual-target))>np.deg2rad(3):raise ValueError('recovery endpoint mismatch')
                report.update(success=True, measured_deg=np.rad2deg(actual).tolist())
                print(json.dumps(report),flush=True)
                break
    finally:
        if handle is not None and handle.accepted:
            cancel=handle.cancel_goal_async()
            end=time.monotonic()+3
            while not cancel.done() and time.monotonic()<end:spin()
            if not cancel.done():print('WARNING: cancellation acknowledgment not received',flush=True)
        (root/'.setup/wrist-recovery.json').write_text(json.dumps(report,indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
