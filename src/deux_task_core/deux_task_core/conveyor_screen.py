"""Finite ZED-observed upper surfaces; hidden geometry is not reconstructed."""
import json
from pathlib import Path
from functools import lru_cache
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull
from deux_task_core.model import JOINT_NAMES

WRIST_GRASP_RADIUS_M = .05
EXTRA_MARGIN_M = .015

@lru_cache(maxsize=1)
def surfaces():
 data=json.loads((Path(__file__).resolve().parents[3]/'calibration/conveyor_visible_surface.json').read_text())
 n=np.array(data['plane']['normal']);n/=np.linalg.norm(n);origin=-data['plane']['d']*n
 e1=np.array([1.,0,0]);e1-=n*(e1@n);e1/=np.linalg.norm(e1);basis=np.stack([e1,np.cross(n,e1)],axis=1)
 result=[]
 for item in data['surfaces']:
  vertices=np.array(item['vertices_base_m']);uv=(vertices-origin)@basis;hull=ConvexHull(uv)
  result.append((vertices,np.roll(vertices,-1,axis=0)-vertices,hull.equations))
 return n,origin,basis,result,float(data['surface_uncertainty_m'])

def point_distances(points):
 n,origin,basis,polygons,_=surfaces();uv=(points-origin)@basis;minimum=np.full(len(points),np.inf)
 for a,ab,equations in polygons:
  inside=np.all(uv@equations[:,:2].T+equations[:,2]<=1e-10,axis=1)
  offset=points[:,None,:]-a[None,:,:]
  t=np.clip(np.sum(offset*ab,axis=2)/np.sum(ab*ab,axis=1),0,1)
  edges=np.min(np.linalg.norm(offset-t[:,:,None]*ab,axis=2),axis=1)
  distance=np.where(inside,abs((points-origin)@n),edges)
  minimum=np.minimum(minimum,distance)
 return minimum

def clearance(robot,q,extra_margin=EXTRA_MARGIN_M):
 values=dict(zip(JOINT_NAMES,q));T=np.eye(4);points=[]
 names={f'arm_r_joint{i}' for i in range(1,8)}
 for name,j in robot.ee_chains['right']:
  M=np.eye(4)
  if j['kind']!='fixed':
   value=values.get(name,robot.fixed.get(name))
   if j['kind'] in ('revolute','continuous'):M[:3,:3]=Rotation.from_rotvec(j['axis']*value).as_matrix()
   else:M[:3,3]=j['axis']*value
  T=T@j['origin']@M
  if name in names:points.append(T[:3,3].copy())
 points.append(T[:3,3].copy());samples=[];deductions=[]
 for i,(a,b) in enumerate(zip(points[:-1],points[1:])):
  length=float(np.linalg.norm(b-a));steps=max(1,int(np.ceil(length/.01)))
  samples.extend(np.linspace(a,b,steps+1))
  # Distance to a closed set is 1-Lipschitz: subtract half sample spacing
  # to bound the entire capsule segment conservatively, not just sample points.
  deductions.extend([(WRIST_GRASP_RADIUS_M if i==6 else .06)+extra_margin+surfaces()[4]+length/(2*steps)]*(steps+1))
 return float(np.min(point_distances(np.array(samples))-deductions))
