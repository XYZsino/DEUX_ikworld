"""Explicit, bounded inward recovery of arm_l_joint6; no general limit override.

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
    import socket, struct, uuid
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from control_msgs.action import FollowJointTrajectory
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from deux_task_msgs.msg import TaskStatus, MotionPlan
    from std_msgs.msg import Bool
    from deux_task_runtime.motion_review import MotionReview, Ticket, preview
    from deux_task_core.model import RobotModel, JOINT_NAMES, ARM_NAMES
    from deux_task_runtime.common import RuntimeNode, LATCHED, current, joint_array, ros_time
    root=Path('/home/xyz_thor/DEUX_ikworld')
    rclpy.init()
    node=RuntimeNode('bounded_left_wrist_recovery', observe_state=True, observe_status=True,
        parameter_overrides=[Parameter('config_file', value=str(root/'src/deux_task_bringup/config/installation.yaml'))])
    state={};estop=[False];external=[]
    review=MotionReview(node)
    can_sockets=[];can_seen={}
    for bus in ('can0','can1'):
        sock=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
        sock.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_FD_FRAMES,1)
        sock.bind((bus,));sock.setblocking(False);can_sockets.append((bus,sock))
    def poll_can():
        for bus,sock in can_sockets:
            while True:
                try:data=sock.recv(72)
                except BlockingIOError:break
                cid=struct.unpack_from('=I',data)[0]
                if cid in range(0x11,0x18) and data[4]>=8:can_seen[(bus,cid)]=time.monotonic()

    subs=[node.create_subscription(JointState, '/arm/joint_states', lambda m:state.update(arm=m), qos_profile_sensor_data),
          node.create_subscription(TaskStatus, '/task/status', lambda m:state.update(task=m), LATCHED)]
    subs.extend([node.create_subscription(Bool,'/deux/estop',lambda m:estop.__setitem__(0,m.data),10),
        node.create_subscription(JointState,'/task/hand_state',lambda m:state.update(hand=m),qos_profile_sensor_data),
        node.create_subscription(JointTrajectory,'/arm/deux_arm_controller/joint_trajectory',lambda m:external.append(time.monotonic()),10)])
    client=ActionClient(node, FollowJointTrajectory, '/arm/deux_arm_controller/follow_joint_trajectory')
    robot=RobotModel(node.config['robot'])
    indices=[JOINT_NAMES.index(n) for n in ARM_NAMES]
    limits=robot.limits[indices]; index=list(ARM_NAMES).index('arm_l_joint6')
    def spin():
        rclpy.spin_once(node, timeout_sec=.02);poll_can()
    def measured():
        m=state.get('arm')
        if m is None or not current(m.header.stamp,node.now(),node.config['max_joint_age']):
            raise ValueError('arm feedback missing/stale')
        return joint_array(m,ARM_NAMES)
    def gate():
        poll_can()
        missing=[(bus,hex(cid)) for bus,_ in can_sockets for cid in range(0x11,0x18) if time.monotonic()-can_seen.get((bus,cid),0)>.2]
        if missing:raise ValueError('CAN feedback stale: '+str(missing))
        if estop[0]:raise ValueError('e-stop active')
        if external and time.monotonic()-external[-1]<.5:raise ValueError('another arm trajectory command')
        # Recovery uses direct fresh arm feedback, independent of object observations.
        node.hardware_gate(); node.check_model_version(); measured()
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
        # Geometry check uses measured hands; hand joint values are never commanded here.
        from deux_task_core.model import HAND_NAMES
        if node.config['obstacles'] or node.config['attached_radius']:
            raise ValueError('recovery requires explicit environment/attachment path checking')
        full=np.tile(np.array(node.config['home_joints']),(len(t),1));full[:,indices]=path
        hand=state.get('hand')
        if hand is None:raise ValueError('hand feedback missing for geometry')
        hands=joint_array(hand,HAND_NAMES);full[:,7:14]=hands[:7];full[:,21:28]=hands[7:]
        for row in full:
            fk=robot.fk(row);spheres=robot.spheres
            for i,a in enumerate(spheres):
                for b in spheres[i+1:]:
                    if frozenset((a['link'],b['link'])) in robot.ignore:continue
                    pa=(fk[a['link']]@np.r_[a['center'],1])[:3];pb=(fk[b['link']]@np.r_[b['center'],1])[:3]
                    if np.linalg.norm(pa-pb)-a['radius']-b['radius']<node.config['collision_margin']:
                        raise ValueError('recovery sphere collision')
        preview_tr=JointTrajectory(joint_names=list(ARM_NAMES),points=[JointTrajectoryPoint(positions=row.tolist(),time_from_start=ros_time(float(ts),Duration)) for ts,row in zip(t,path)])
        plan=MotionPlan(plan_id=uuid.uuid4().hex,stage='LeftJ6Recovery',scene_version=node.config_version,trajectory=preview_tr)
        review.data=preview(robot,plan,full[0],t,full)
        review.ticket=Ticket(plan.plan_id,review.data['digest'],300.)
        review.data.update(request_id=review.ticket.request_id,warning=f'Left J6 inward recovery: {np.degrees(start[index]):.3f} -> {np.degrees(target[index]):.3f} deg, 4 seconds. Other arms held. No hand commands.')
        print('AWAITING_GUI_APPROVAL',flush=True)
        while review.ticket.decision is None:
            gate();review.publish('pending','Left J6 inward recovery');spin()
            if time.monotonic()>review.ticket.deadline:raise ValueError('preview expired')
        if review.ticket.decision!='approve':raise ValueError('operator rejected recovery')
        review.publish('approved','Left J6 inward recovery');review.ticket=None;review.data=None
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
        if review.ticket is not None:review.publish('closed','recovery finished or stopped')
        for _,sock in can_sockets:sock.close()
        (root/'.setup/left-wrist-recovery.json').write_text(json.dumps(report,indent=2))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
