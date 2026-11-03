from right_only_trial import planning_robot
"""Generate a collision-screened return path from freshly measured joints."""
import time
import numpy as np
from deux_task_core.model import RobotModel
from deux_task_core.conveyor_screen import clearance

def compute_ready(config,current,goal):
 r=planning_robot(config);current=np.array(current);goal=np.array(goal);start=current[14:21];finish=goal[14:21];lo=r.limits[14:21,0]+.04;hi=r.limits[14:21,1]-.04
 def clear(arm):
  q=current.copy();q[14:21]=arm
  try:r.clearance(q,config['obstacles'],config['collision_margin'])
  except ValueError:return False
  return clearance(r,q)>=0
 def edge(a,b,step=.04):return all(clear(q) for q in np.linspace(a,b,max(3,int(np.ceil(max(abs(b-a))/step))+1)))
 if not clear(start) or not clear(finish):raise ValueError('ready recovery endpoint collision/limit')
 if edge(start,finish):path=[start,finish]
 else:
  rng=np.random.default_rng(31);trees=[([start],[-1]),([finish],[-1])];path=None;began=time.monotonic()
  for it in range(3000):
   side=it%2;nodes,parents=trees[side];other,op=trees[1-side]
   sample=other[rng.integers(len(other))] if rng.random()<.25 else np.clip(start+(finish-start)*rng.random()+rng.normal(0,.9,7),lo,hi)
   near=int(np.argmin(np.linalg.norm(np.array(nodes)-sample,axis=1)));d=sample-nodes[near];new=nodes[near]+d*min(1.,.25/max(np.linalg.norm(d),1e-9))
   if not edge(nodes[near],new):continue
   nodes.append(new);parents.append(near);j=int(np.argmin(np.linalg.norm(np.array(other)-new,axis=1)))
   if np.linalg.norm(other[j]-new)<.5 and edge(other[j],new):
    def branch(ns,pp,i):
     out=[]
     while i>=0:out.append(ns[i]);i=pp[i]
     return out[::-1]
    a=branch(nodes,parents,len(nodes)-1);b=branch(other,op,j);path=a+b[::-1] if side==0 else b+a[::-1];break
   if time.monotonic()-began>12:break
  if path is None:raise ValueError('ready recovery path unavailable')
  for _ in range(100):
   if len(path)<3:break
   i,j=sorted(rng.choice(len(path),2,replace=False))
   if j>i+1 and edge(path[i],path[j],.025):path=path[:i+1]+path[j:]
 for a,b in zip(path[:-1],path[1:]):
  if not edge(a,b,.004):raise ValueError('dense ready recovery collision')
 return dict(right_waypoints=[q.tolist() for q in path],screening=dict(surface_model="calibration/conveyor_visible_surface.json",surface_uncertainty_m=.0175,arm_segment_radius_m=.06,wrist_grasp_radius_m=.05,extra_margin_m=.015,geometry='provisional skeleton capsules'))
