"""Recheck the proposal as a continuation of the original ready-to-contact path."""
import sys,json
from pathlib import Path
import numpy as np,yaml
from scipy.interpolate import make_interp_spline
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'tools'),str(ROOT/'src/deux_task_core')]
from classified_grid_runtime import ClassifiedGridLibrary
from lookup_conveyor import validate_conveyor_path
out=ROOT/'artifacts/right-high-scan-waypoint';p=ROOT/'src/deux_task_bringup/config/installation.yaml';c=yaml.safe_load(p.read_text());c['robot']['urdf_path']=str((p.parent/c['robot']['urdf_path']).resolve())
lib=ClassifiedGridLibrary(c,directory=ROOT/'artifacts/right_grasp_grid_classified',recommended=False,yaw_secondary=True,load_rotation_paths=False)
d=json.loads((out/'proposal.json').read_text());q=np.asarray(d['positions_rad']);t=np.asarray(d['times_s']);robot=lib.robot
prefix_t=np.linspace(0.,3.,151);prefix=np.tile(q[0],(len(prefix_t),1));prefix[:,14:21]=lib.curves[d['source_cell']](prefix_t)
whole=np.vstack([prefix,q[1:]]);time=np.r_[prefix_t,3.+t[1:]]
assert np.max(abs(prefix[-1]-q[0]))<1e-9
assert np.max(abs(q[-1,14:21]-c['scan_right_joints']))<1e-10
assert np.all(whole>=robot.limits[:,0]-robot.joint_limit_tolerance) and np.all(whole<=robot.limits[:,1]+robot.joint_limit_tolerance)
dt=np.diff(time);v=np.diff(whole,axis=0)/dt[:,None];a=np.diff(np.vstack([np.zeros(28),v,np.zeros(28)]),axis=0)/np.r_[dt,dt[-1]][:,None]
vr=float(np.max(abs(v)/robot.velocity));ar=float(np.max(abs(a)/robot.acceleration))
assert vr<=1.001 and ar<=1.01,(vr,ar)
margin=validate_conveyor_path(robot,q)
vi=np.argmin(abs(t-d['waypoint_at_s']));via=robot.ee_fk('right',q[vi])[:3,3]
assert np.linalg.norm(via-d['waypoint_xyz_m'])<.003
assert via[1]<d['lifted_stop_xyz_m'][1]-.05 and via[2]>d['lifted_stop_xyz_m'][2]+.15
assert .015<d['scan_xyz_m'][2]-via[2]<.045
assert np.max(np.asarray(d['ee_path_m'])[:,2])<=d['scan_xyz_m'][2]+.0005
assert d['minimum_z_after_moving_toward_scan_m']>=1.21
assert np.min(np.asarray(d['ee_path_m'])[vi:,2])>=via[2]-.004
way_speed=float(np.linalg.norm((q[vi+1,14:21]-q[vi-1,14:21])/(t[vi+1]-t[vi-1])))
assert way_speed>.02,'waypoint must not stop'
assert t[-1]<d['original_transfer_duration_s']
join=d['join_original_trajectory_at_s'];relative_join=join-3.;ji=int(np.argmin(abs(t-relative_join)))
assert abs(t[ji]-relative_join)<1e-8
v0=np.zeros(28);a0=np.zeros(28);v0[14:21]=lib.curves[d['source_cell']](join,1);a0[14:21]=lib.curves[d['source_cell']](np.nextafter(join,-np.inf),2)
curve=make_interp_spline([0.,t[vi]-relative_join,t[-1]-relative_join],q[[ji,vi,-1]],k=5,bc_type=([(1,v0),(2,a0)],[(1,np.zeros(28)),(2,np.zeros(28))]))
assert np.max(abs(curve(t[ji:]-relative_join)-q[ji:]))<1e-7
velocity_join_error=float(np.max(abs(curve(0.,nu=1)-v0)));acceleration_join_error=float(np.max(abs(curve(0.,nu=2)-a0)))
assert velocity_join_error<1e-7 and acceleration_join_error<1e-7
result=dict(checked_source_cell=d['source_cell'],ready_to_scan_duration_s=float(time[-1]),contact_at_original_s=3.,new_lift_handoff_at_original_s=d['join_original_trajectory_at_s'],contact_and_lift_prefix_unchanged=True,
 velocity_join_error_rad_s=velocity_join_error,acceleration_join_error_rad_s2=acceleration_join_error,full_trajectory_velocity_limit_ratio=vr,full_trajectory_acceleration_limit_ratio=ar,minimum_conveyor_clearance_m=margin,
 waypoint_xyz_m=via.tolist(),waypoint_joint_speed_norm_rad_s=way_speed,waypoint_s_after_contact=float(t[vi]),new_motion_duration_after_contact_s=float(t[-1]),
 actual_robot_commands=False,dynamic_bread_bag_collision_certified=False)
(out/'validation.json').write_text(json.dumps(result,indent=2)+'\n')
(out/'ready-to-scan-proposal.json').write_text(json.dumps(dict(times_s=time.tolist(),positions_rad=whole.tolist(),preview_only=True),separators=(',',':'))+'\n')
print(json.dumps(result,indent=2))
