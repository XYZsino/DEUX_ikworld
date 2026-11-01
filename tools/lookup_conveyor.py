"""Lookup transfers: arm capsules against the measured front conveyor only.

Joint bounds remain enforced. No inter-arm/self/scene-box collision tests.
Uses the same provisional capsule radii and surface uncertainty as stored paths.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from deux_task_core.model import JOINT_NAMES
from deux_task_core.conveyor_screen import point_distances,surfaces,WRIST_GRASP_RADIUS_M,EXTRA_MARGIN_M


def arm_clearance(robot,q,side):
    values=dict(zip(JOINT_NAMES,q));transform=np.eye(4);points=[]
    prefix='arm_l_joint' if side=='left' else 'arm_r_joint'
    for name,joint in robot.ee_chains[side]:
        motion=np.eye(4)
        if joint['kind']!='fixed':
            value=values.get(name,robot.fixed.get(name))
            if joint['kind'] in ('revolute','continuous'):
                motion[:3,:3]=Rotation.from_rotvec(joint['axis']*value).as_matrix()
            else:motion[:3,3]=joint['axis']*value
        transform=transform@joint['origin']@motion
        if name in {prefix+str(i) for i in range(1,8)}:points.append(transform[:3,3].copy())
    points.append(transform[:3,3].copy());samples=[];deductions=[]
    for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
        length=float(np.linalg.norm(b-a));steps=max(1,int(np.ceil(length/.01)))
        samples.extend(np.linspace(a,b,steps+1))
        radius=WRIST_GRASP_RADIUS_M if i==6 else .06
        deductions.extend([radius+EXTRA_MARGIN_M+surfaces()[4]+length/(2*steps)]*(steps+1))
    return float(np.min(point_distances(np.asarray(samples))-deductions))


def validate_conveyor_path(robot,path,canceled=lambda:False):
    points=np.asarray(path,float)
    if points.ndim!=2 or points.shape[1]!=28 or not np.isfinite(points).all():raise ValueError('invalid conveyor path')
    if np.any(points<robot.limits[:,0]-robot.joint_limit_tolerance) or np.any(points>robot.limits[:,1]+robot.joint_limit_tolerance):raise ValueError('joint limit')
    minimum=float('inf')
    for side,indices in [('left',slice(0,7)),('right',slice(14,21))]:
        radius=sum(np.linalg.norm(j['origin'][:3,3]) for _,j in robot.ee_chains[side])+.01
        def edge(a,b,ca,cb,depth=0):
            if canceled():raise InterruptedError('planning canceled')
            if min(ca,cb)<0:raise ValueError(f'{side} arm conveyor collision')
            bound=.5*radius*np.sum(abs(b[indices]-a[indices]))
            if min(ca,cb)-bound>=0:return min(ca,cb)-bound
            if depth>=16:raise ValueError(f'{side} conveyor clearance not certified')
            middle=(a+b)/2;cm=arm_clearance(robot,middle,side)
            return min(edge(a,middle,ca,cm,depth+1),edge(middle,b,cm,cb,depth+1))
        ca=arm_clearance(robot,points[0],side)
        if ca<0:raise ValueError(f'{side} arm conveyor collision')
        minimum=min(minimum,ca)
        for a,b in zip(points[:-1],points[1:]):
            cb=arm_clearance(robot,b,side);minimum=min(minimum,edge(a,b,ca,cb));ca=cb
    return minimum
