"""Offline hand geometry from user angles. No ROS publishers or hardware access."""
from pathlib import Path
import json, hashlib, xml.etree.ElementTree as ET
import numpy as np
import yaml
import trimesh
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
from deux_task_core.model import origin, PARTS

R=Path(__file__).resolve().parents[1]
URDF=R/'src/deux_task_bringup/urdf/deux_thor_measured_lift.urdf'
root=ET.parse(URDF).getroot()
by_child={j.find('child').get('link'):j for j in root.findall('joint')}
by_name={j.get('name'):j for j in root.findall('joint')}
links={l.get('name'):l for l in root.findall('link')}
poses={
 'right':{'initial':[0.,0.,0.,0.,0.,0.,0.],'target':[0.,-80.,20.,130.,-50.,-130.,50.]},
 'left':{'initial':[0.,32.6,64.2,-53.8,-45.,90.,62.],'target':[0.,32.6,64.2,-78.,-45.,130.,62.]}}
def fk(side,link,angles):
 q=dict(zip([side+'_hand_'+s for s in PARTS],np.deg2rad(angles)))
 t=np.eye(4)
 while link!=side+'_hand_base_link':
  j=by_child[link];part=origin(j.find('origin'))
  if j.get('name') in q:
   axis=np.array(j.find('axis').get('xyz').split(),float)
   motion=np.eye(4);motion[:3,:3]=Rotation.from_rotvec(axis*q[j.get('name')]).as_matrix()
   part=part@motion
  t=part@t;link=j.find('parent').get('link')
 return t
def distal(side,finger,angles):
 link=side+'_hand_'+finger
 v=links[link].find('collision');m=v.find('geometry/mesh')
 path=Path('/home/xyz_thor/DEUX/install/deux_description/share/deux_description')/m.get('filename').split('package://deux_description/')[1]
 mesh=trimesh.load_mesh(path,process=False)
 # Mesh units and origin are taken from URDF, not assumed from STL.
 points=np.vstack((mesh.vertices,mesh.triangles_center))*np.array(m.get('scale','1 1 1').split(),float)
 tr=origin(v.find('origin'));points=points@tr[:3,:3].T+tr[:3,3]
 axis=1 if finger.startswith('thumb') else 2
 extent=points[:,axis].max()-points[:,axis].min()
 cap=points[points[:,axis]>=points[:,axis].max()-.005]
 tip=cap.mean(axis=0)
 terminal=points[points[:,axis]>=points[:,axis].min()+.5*extent]
 t=fk(side,link,angles)
 return dict(link=link,mesh=str(path),tip_local_m=tip.tolist(),
             tip_hand_m=(t[:3,:3]@tip+t[:3,3]).tolist(),
             T_hand_link=t.tolist()),terminal@t[:3,:3].T+t[:3,3]
out={'schema':'deux.hand_geometry.v1','units':'m/rad','urdf':str(URDF),
 'urdf_sha256':hashlib.sha256(URDF.read_bytes()).hexdigest(),
 'joint_order':list(PARTS),'user_order_and_degrees_confirmed':True,
 'right_contact_pair':'thumb/index','hardware_motor_mapping_confirmed':False,
 'physical_contact_calibrated':False,'reader_optical_transform_known':False,
 'method':'URDF FK; distal tip = mean of mesh vertices and triangle centers in final 5 mm along local extension axis; opposing candidate = closest sampled distal-half surfaces, not measured contact',
 'hands':{}}
for side,states in poses.items():
 out['hands'][side]={}
 for state,angles in states.items():
  rad=np.deg2rad(angles);violations=[]
  for part,a in zip(PARTS,rad):
   lim=by_name[side+'_hand_'+part].find('limit')
   # URDF limits rounded to 6 decimals; 130 deg differs by <1e-6 rad.
   if not float(lim.get('lower'))-1e-6<=a<=float(lim.get('upper'))+1e-6:violations.append(part)
  assert not violations,(side,state,violations)
  details={};clouds={}
  for finger in ('thumb_3','index_2','third_2'):
   details[finger],clouds[finger]=distal(side,finger,angles)
  distances,indices=cKDTree(clouds['index_2']).query(clouds['thumb_3'])
  k=int(np.argmin(distances));a=clouds['thumb_3'][k];b=clouds['index_2'][indices[k]]
  tip_a=np.array(details['thumb_3']['tip_hand_m']);tip_b=np.array(details['index_2']['tip_hand_m'])
  out['hands'][side][state]={'degrees':angles,'radians':rad.tolist(),'within_urdf_limits_1e6_tolerance':True,
   'distal_links':details,'thumb_index_tip_midpoint_m':((tip_a+tip_b)/2).tolist(),
   'thumb_index_tip_distance_m':float(np.linalg.norm(tip_a-tip_b)),
   'closest_distal_surface_pair_m':[a.tolist(),b.tolist()],
   'closest_distal_surface_midpoint_m':((a+b)/2).tolist(),
   'sampled_surface_separation_m':float(distances[k])}
# Do not silently turn a guessed contact or optical frame into the planning EE.
dest=R/'calibration/hand_geometry.thor.json'
dest.write_text(json.dumps(out,indent=2)+'\n')
profile={'schema':'deux.hand_postures.v1','units':'rad','joint_order':list(PARTS),
         'hardware_mapping_confirmed':False,'hands':{}}
for side in poses:
 profile['hands'][side]={'initial':np.deg2rad(poses[side]['initial']).tolist(),
                         'target':np.deg2rad(poses[side]['target']).tolist()}
(R/'calibration/hand_postures.thor.yaml').write_text(yaml.safe_dump(profile,sort_keys=False))
# Fill only explicitly supplied postures, ordered left seven then right seven.
path=R/'src/deux_task_bringup/config/installation.yaml'
s=path.read_text()
for key,state in [('hand_open','initial'),('hand_closed','target')]:
 left_state='target' if yaml.safe_load((R/'src/deux_task_bringup/config/hand_mapping.yaml').read_text()).get('keep_left_closed',False) else state
 values=np.deg2rad(poses['left'][left_state]+poses['right'][state]).tolist()
 lines=s.splitlines()
 for i,line in enumerate(lines):
  if line.startswith(key+':'):lines[i]=key+': '+json.dumps(values)
 s='\n'.join(lines)+'\n'
path.write_text(s)
for side in poses:
 for state,v in out['hands'][side].items():
  print(side,state,json.dumps({k:v[k] for k in ['thumb_index_tip_midpoint_m','thumb_index_tip_distance_m','closest_distal_surface_midpoint_m','sampled_surface_separation_m']}))
print('Saved',dest)
