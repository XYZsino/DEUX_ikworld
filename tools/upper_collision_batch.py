from deux_task_core.conveyor_screen import WRIST_GRASP_RADIUS_M, EXTRA_MARGIN_M
"""Batch evaluation of unchanged RobotModel and finite-conveyor clearance rules."""
import numpy as np
from scipy.spatial.transform import Rotation
from deux_task_core.model import JOINT_NAMES
from deux_task_core.conveyor_screen import surfaces,point_distances


def checked_clearances(robot,states,obstacles,margin,extra_margin=EXTRA_MARGIN_M):
 q=np.asarray(states,dtype=float)
 if q.ndim!=2 or q.shape[1]!=28 or not len(q) or not np.isfinite(q).all():raise ValueError('invalid batched states')
 if np.any(q<robot.limits[:,0]-robot.joint_limit_tolerance) or np.any(q>robot.limits[:,1]+robot.joint_limit_tolerance):raise ValueError('joint limit')
 count=len(q);values=dict(zip(JOINT_NAMES,q.T));eye=np.broadcast_to(np.eye(4),(count,4,4));links={robot.base:eye}
 def visit(parent):
  for name in robot.children.get(parent,[]):
   j=robot.joints[name];motion=eye.copy()
   if j['kind']!='fixed':
    if name not in values and name not in robot.fixed:raise ValueError('unmeasured joint: '+name)
    value=np.broadcast_to(values.get(name,robot.fixed.get(name)),(count,))
    if not np.isfinite(value).all():raise ValueError('invalid fixed joint')
    if j['kind'] in ('revolute','continuous'):motion[:,:3,:3]=Rotation.from_rotvec(value[:,None]*j['axis']).as_matrix()
    elif j['kind']=='prismatic':motion[:,:3,3]=value[:,None]*j['axis']
    else:raise ValueError('unsupported joint type')
   links[j['child']]=links[parent]@j['origin']@motion;visit(j['child'])
 visit(robot.base)
 if robot.collision_check_enabled:
  spheres=[(s['link'],(links[s['link']]@np.r_[s['center'],1])[:,:3],float(s['radius'])) for s in robot.spheres]
  for i,(name,center,radius) in enumerate(spheres):
   for other,other_center,other_radius in spheres[:i]:
    if name==other or frozenset((name,other)) in robot.ignore:continue
    if np.any(np.linalg.norm(center-other_center,axis=1)-radius-other_radius<margin):raise ValueError('collision clearance')
   for box in obstacles:
    lo,hi=np.array(box['min']),np.array(box['max'])
    if lo.shape!=(3,) or hi.shape!=(3,) or not np.isfinite([lo,hi]).all():raise ValueError('invalid obstacle')
    if np.any(np.linalg.norm(center-np.clip(center,lo,hi),axis=1)-radius<margin):raise ValueError('collision clearance')
 points=[links[robot.joints[f'arm_r_joint{i}']['child']][:,:3,3] for i in range(1,8)]
 points.append(links[robot.ee['right']][:,:3,3]);minimum=np.full(count,np.inf)
 for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
  lengths=np.linalg.norm(b-a,axis=1);steps=np.maximum(1,np.ceil(lengths/.01).astype(int))
  for number in np.unique(steps):
   indices=np.flatnonzero(steps==number);u=np.linspace(0,1,int(number)+1)
   positions=a[indices,None,:]+u[None,:,None]*(b-a)[indices,None,:]
   flat=positions.reshape(-1,3);dist=np.concatenate([point_distances(flat[k:k+2048]) for k in range(0,len(flat),2048)])
   distance=dist.reshape(len(indices),len(u)).min(axis=1)
   margin_values=distance-(WRIST_GRASP_RADIUS_M if i==6 else .06)-extra_margin-surfaces()[4]-lengths[indices]/(2*number)
   minimum[indices]=np.minimum(minimum[indices],margin_values)
 if np.any(minimum<0):raise ValueError('continuous provisional conveyor collision')
 return minimum
