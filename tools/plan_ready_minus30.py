from pathlib import Path
import json,time
import numpy as np,yaml
from scipy.spatial.transform import Rotation
from deux_task_core.configuration import load_configuration
from deux_task_core.model import RobotModel,JOINT_NAMES
root=Path('/home/xyz_thor/DEUX_ikworld');out=root/'.setup/ready-minus30-path';out.mkdir(exist_ok=True)
c,_=load_configuration(str(root/'src/deux_task_bringup/config/installation.yaml'));r=RobotModel(c['robot']);base=np.array(c['home_joints'])
raw=(root/'.setup/ready-arm-trial/current-joints.yaml').read_text();s=next(yaml.safe_load_all('header:'+raw.split('header:',1)[1]))
for k,v in zip(s['name'],s['position']):base[JOINT_NAMES.index(k)]=v
start=base[14:21].copy();goal=np.array(json.loads((root/'.setup/ready-minus30/candidate.json').read_text())['chosen']['new_home'])[14:21]
# Conservative screening model, not calibrated robot collision geometry:
# conveyor is solid below .92m and everywhere ahead of X=.368m (extends beyond camera FOV).
# Arm segments 6cm radius, wrist-to-grasp envelope 10cm, plus 2cm clearance.
chain=r.ee_chains['right'];names=[f'arm_r_joint{i}' for i in range(1,8)]
def points(q):
 values=dict(zip(JOINT_NAMES,base));values.update(zip(names,q));T=np.eye(4);ps=[]
 for name,j in chain:
  M=np.eye(4)
  if j['kind']!='fixed':
   v=values.get(name,r.fixed.get(name))
   if j['kind'] in ('revolute','continuous'):M[:3,:3]=Rotation.from_rotvec(j['axis']*v).as_matrix()
   else:M[:3,3]=j['axis']*v
  T=T@j['origin']@M
  if name in names:ps.append(T[:3,3].copy())
 ps.append(T[:3,3].copy());return np.array(ps)
def clearance(q):
 ps=points(q);best=10.
 for i,(a,b) in enumerate(zip(ps[:-1],ps[1:])):
  p=a+(b-a)*np.linspace(0,1,13)[:,None]
  # distance to infinite X>=.368, Z<=.92 solid quadrant
  dist=np.hypot(np.maximum(.368-p[:,0],0),np.maximum(p[:,2]-.92,0))
  best=min(best,float(dist.min())-(.10 if i==6 else .06)-.03)
 return best
def edge(a,b,step=.055):
 qs=np.linspace(a,b,max(3,int(np.ceil(max(abs(b-a))/step))+1));mi=10.
 for q in qs:
  v=clearance(q);mi=min(mi,v)
  if v<0:return False,mi
 return True,mi
beg=time.monotonic();print('start/goal clearance',clearance(start),clearance(goal),flush=True);print('direct',edge(start,goal),flush=True)
rng=np.random.default_rng(31);limits=r.limits[14:21];valid=[]
# Bidirectional joint-space search with conservative obstacle screening.
trees=[([start.copy()],[-1]),([goal.copy()],[-1])];found=None
for iteration in range(6000):
 side=iteration%2;nodes,parents=trees[side];other,op=trees[1-side]
 if rng.random()<.25:sample=other[rng.integers(len(other))]
 else:
  sample=start+(goal-start)*rng.random()+rng.normal(0,.9,7)
  sample=np.clip(sample,limits[:,0]+.04,limits[:,1]-.04)
 near=int(np.argmin(np.linalg.norm(np.array(nodes)-sample,axis=1)));d=sample-nodes[near];new=nodes[near]+d*min(1.,.25/max(np.linalg.norm(d),1e-9))
 if not edge(nodes[near],new,.07)[0]:continue
 nodes.append(new);parents.append(near);j=int(np.argmin(np.linalg.norm(np.array(other)-new,axis=1)))
 if np.linalg.norm(other[j]-new)<.5 and edge(other[j],new,.055)[0]:
  def branch(ns,pp,i):
   out=[]
   while i>=0:out.append(ns[i]);i=pp[i]
   return out[::-1]
  aa=branch(nodes,parents,len(nodes)-1);bb=branch(other,op,j)
  found=aa+bb[::-1] if side==0 else bb+aa[::-1];break
 if time.monotonic()-beg>110:break
if found is None:raise RuntimeError('no conservative path found '+str([len(t[0]) for t in trees]))
# Shortcut only if full interpolated edge remains clear.
for _ in range(120):
 if len(found)<3:break
 i,j=sorted(rng.choice(len(found),2,replace=False))
 if j>i+1 and edge(found[i],found[j],.03)[0]:found=found[:i+1]+found[j:]
waypoints=found
# Dense final collision screen at <=0.25 degree joint increments, normal configuration too.
minimum=10.;frames=[]
for a,b in zip(waypoints[:-1],waypoints[1:]):
 ok,d=edge(a,b,.004);assert ok;minimum=min(minimum,d)
 for arm in np.linspace(a,b,60):
  q=base.copy();q[14:21]=arm;r.clearance(q,c['obstacles'],c['collision_margin']);frames.append(points(arm).tolist())
report=dict(start_full=base.tolist(),right_waypoints=[q.tolist() for q in waypoints],waypoint_deg=[np.degrees(q).tolist() for q in waypoints],min_screening_clearance_m=minimum,screening=dict(conveyor_front_x_m=.368,conveyor_top_z_m=.92,arm_segment_radius_m=.06,wrist_grasp_radius_m=.10,extra_margin_m=.03,y_extent='unbounded',geometry='provisional skeleton capsules, not measured mesh'),search_iterations=iteration,waypoint_count=len(waypoints),elapsed_s=time.monotonic()-beg,frames=frames)
(out/'path.json').write_text(json.dumps(report,indent=2));print({k:v for k,v in report.items() if k not in ['frames','start_full','right_waypoints']},flush=True)
