"""Opt-in conveyor-checked preparation route, with stopped intermediate knots."""
import numpy as np
from .conveyor_screen import clearance

def knots(config,start,goal,robot,canceled=lambda:False):
 import time
 route=config['prepare_conveyor_route'];way=np.asarray(route['right_waypoints'],float)
 if way.ndim!=2 or way.shape[1]!=7 or len(way)<2 or not np.isfinite(way).all():
  raise ValueError('invalid prepare conveyor route')
 if np.max(abs(way[-1]-goal[14:21]))>1e-6:
  raise ValueError('prepare conveyor route target differs from home')
 start=np.asarray(start,float);goal=np.asarray(goal,float)
 def full(arm):
  q=start.copy();q[14:21]=arm;return q
 def edge(a,b):
  if canceled():raise InterruptedError('planning canceled')
  try:
   qa,qb=full(a),full(b)
   robot.clearance(qa,config['obstacles'],config['collision_margin'])
   robot.clearance(qb,config['obstacles'],config['collision_margin'])
   validate_path(robot,[qa,qb],canceled)
   for t in np.linspace(0,1,max(3,int(np.ceil(np.max(abs(b-a))/.025))+1)):
    robot.clearance(full(a+t*(b-a)),config['obstacles'],config['collision_margin'])
   return True
  except ValueError:return False
 initial=start[14:21];finish=goal[14:21]
 if not edge(initial,initial) or not edge(finish,finish):
  raise ValueError('prepare current/goal pose fails collision or joint check')
 path=None
 if edge(initial,finish):path=[initial,finish]
 else:
  # Try joining the stored safe route, starting with the shortest suffix.
  for i in reversed(range(len(way)-1)):
   candidate=[initial,*way[i:]]
   if all(edge(a,b) for a,b in zip(candidate[:-1],candidate[1:])):
    path=candidate;break
 if path is None:
  rng=np.random.default_rng(91);trees=[([initial],[-1]),([finish],[-1])]
  lo=robot.limits[14:21,0]+.04;hi=robot.limits[14:21,1]-.04
  began=time.monotonic()
  for iteration in range(2500):
   if canceled():raise InterruptedError('planning canceled')
   if time.monotonic()-began>12:break
   side=iteration%2;nodes,parents=trees[side];other,op=trees[1-side]
   sample=other[rng.integers(len(other))] if rng.random()<.3 else np.clip(initial+(finish-initial)*rng.random()+rng.normal(0,.8,7),lo,hi)
   i=int(np.argmin(np.linalg.norm(np.array(nodes)-sample,axis=1)))
   delta=sample-nodes[i];new=nodes[i]+delta*min(1.,.25/max(np.linalg.norm(delta),1e-9))
   if not edge(nodes[i],new):continue
   nodes.append(new);parents.append(i)
   j=int(np.argmin(np.linalg.norm(np.array(other)-new,axis=1)))
   if np.linalg.norm(other[j]-new)<.6 and edge(new,other[j]):
    def branch(ns,pp,k):
     result=[]
     while k>=0:result.append(ns[k]);k=pp[k]
     return result[::-1]
    aa=branch(nodes,parents,len(nodes)-1);bb=branch(other,op,j)
    path=aa+bb[::-1] if side==0 else bb+aa[::-1];break
  if path is None:raise ValueError('no collision-free preparation route from current pose')
 result=[full(arm) for arm in path]
 if np.max(abs(result[-1]-goal))>1e-9:result.append(goal.copy())
 return result

def validate_path(robot,path,canceled=lambda:False):
 # Radius exceeds every rotational lever arm on the right EE chain.
 radius=sum(np.linalg.norm(j['origin'][:3,3]) for _,j in robot.ee_chains['right'])+.01
 minimum=float('inf')
 def edge(a,b,ca,cb,depth=0):
  if canceled():raise InterruptedError('planning canceled')
  if min(ca,cb)<0:raise ValueError('prepare conveyor collision')
  bound=.5*radius*np.sum(abs(b[14:21]-a[14:21]))
  if min(ca,cb)-bound>=0:return min(ca,cb)-bound
  if depth>=16:raise ValueError('prepare conveyor clearance could not be certified')
  m=(a+b)/2;cm=clearance(robot,m)
  return min(edge(a,m,ca,cm,depth+1),edge(m,b,cm,cb,depth+1))
 ca=clearance(robot,path[0])
 for a,b in zip(path[:-1],path[1:]):
  cb=clearance(robot,b);minimum=min(minimum,edge(a,b,ca,cb));ca=cb
 return minimum
