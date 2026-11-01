"""Offline right/high scan waypoint search. No ROS, drivers, or commands."""
import sys,json,time
from pathlib import Path
import numpy as np,yaml
from scipy.spatial.transform import Rotation,Slerp
from scipy.interpolate import CubicHermiteSpline,make_interp_spline
from scipy.optimize import least_squares
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/deux_task_core'),str(ROOT/'tools')]
from classified_grid_runtime import ClassifiedGridLibrary
from deux_task_core.planning import hermite
from lookup_conveyor import validate_conveyor_path


def main():
    out=ROOT/'artifacts/right-high-scan-waypoint';out.mkdir(parents=True,exist_ok=True)
    p=ROOT/'src/deux_task_bringup/config/installation.yaml';c=yaml.safe_load(p.read_text());c['robot']['urdf_path']=str((p.parent/c['robot']['urdf_path']).resolve())
    lib=ClassifiedGridLibrary(c,directory=ROOT/'artifacts/right_grasp_grid_classified',recommended=False,yaw_secondary=True,load_rotation_paths=False)
    key=json.loads((ROOT/'artifacts/thor-lookup-bt-first-run/result.json').read_text())['cell_id']
    robot=lib.robot;robot.ik_position_tolerance_m=.003;robot.ik_rotation_tolerance_rad=np.deg2rad(2.)
    start=lib.ready.copy();start[:7]=c['home_joints'][:7];start[14:21]=lib.curves[key](5.)
    scan=start.copy();scan[14:21]=c['scan_right_joints'];initial=robot.ee_fk('right',start);final=robot.ee_fk('right',scan)
    rotations=Slerp([0.,1.],Rotation.from_matrix([initial[:3,:3],final[:3,:3]]))
    scale=.8;rows=[];began=time.monotonic();candidates=[]
    targets=[[x,y,z] for z in [1.235,1.24,1.245] for y in [-.26,-.3,-.34] for x in [.63,.6,.66]]
    for number,xyz in enumerate(targets):
        for blend in [0.,.5,.8,1.]:
            target=np.eye(4);target[:3,3]=xyz;target[:3,:3]=rotations(blend).as_matrix()
            try:
                q=robot.ik('right',target,start,continuity_weight=.0001)
                t,path=hermite([start,q,scan],robot.velocity*scale,robot.acceleration*scale**2,.02)
                points=np.array([robot.ee_fk('right',row)[:3,3] for row in path])
                index=int(np.argmin(np.linalg.norm(path-q,axis=1)))
                first=points[:index+1];crossing=points[points[:,1]>initial[1,3]+.05]
                # The actual C2 contact-to-scan path below is checked directly.
                score=float(t[-1]+2*np.linalg.norm(np.array(xyz)-[.63,-.32,1.24])+blend*.4)
                candidates.append((score,dict(xyz=xyz,orientation_blend=blend,q=q,t=t,path=path,ee=points,way_index=index)))
            except ValueError as error:rows.append(dict(xyz=xyz,orientation_blend=blend,rejected=str(error)))
        if number%6==0:print(json.dumps(dict(targets_checked=number+1,total=len(targets),candidates=len(candidates))),flush=True)
    # The intermediate transport pose need not rigidly copy the scan rotation.
    # Prefer a continuous joint posture while softly retaining hand orientation.
    reference=(start[14:21]+scan[14:21])*.5
    for xyz in [[x,y,z] for x in [.63,.66] for y in [-.26,-.3] for z in [1.235,1.24]]:
        for blend in [.5,.8]:
            desired_rotation=rotations(blend).as_matrix()
            def residual(arm):
                trial=start.copy();trial[14:21]=arm;pose=robot.ee_fk('right',trial)
                return np.r_[pose[:3,3]-xyz,.02*Rotation.from_matrix(desired_rotation@pose[:3,:3].T).as_rotvec(),.002*(arm-reference)]
            fit=least_squares(residual,np.clip(reference,robot.limits[14:21,0]+.04,robot.limits[14:21,1]-.04),bounds=(robot.limits[14:21,0]+.035,robot.limits[14:21,1]-.035),max_nfev=180)
            q=start.copy();q[14:21]=fit.x
            if np.linalg.norm(robot.ee_fk('right',q)[:3,3]-xyz)>.003:continue
            candidates.append((0.,dict(xyz=xyz,orientation_blend=blend,q=q,orientation_soft=True)))
    # Keep a short piece of the original lift to preserve conveyor-following
    # contact velocity. Search the earliest smooth handoff into the high exit.
    chosen=None
    for join_at in [3.25,3.5,3.75,4.,4.25,4.45,4.5,4.75,5.]:
        contact=start.copy();contact[14:21]=lib.curves[key](join_at)
        contact_v=np.zeros(28);contact_v[14:21]=lib.curves[key](join_at,1)
        contact_pose=robot.ee_fk('right',contact);valid_direct=[]
        for score,data in sorted(candidates,key=lambda x:x[0]):
            q=data['q'];bt,bp=hermite([contact,q,scan],robot.velocity*scale,robot.acceleration*scale**2,.02)
            wi=int(np.argmin(np.linalg.norm(bp-q,axis=1)));dur=np.array([bt[wi],bt[-1]-bt[wi]]);knots=np.array([contact,q,scan])
            for _ in range(12):
                tk=np.r_[0.,np.cumsum(dur)];slopes=np.diff(knots,axis=0)/dur[:,None]
                tangent=np.zeros_like(knots);tangent[0]=contact_v;same=slopes[0]*slopes[1]>0
                tangent[1,same]=2/(1/slopes[0,same]+1/slopes[1,same])
                acceleration0=np.zeros(28);acceleration0[14:21]=lib.curves[key](np.nextafter(join_at,-np.inf),2)
                curve=make_interp_spline(tk,knots,k=5,bc_type=([(1,contact_v),(2,acceleration0)],[(1,np.zeros(28)),(2,np.zeros(28))]))
                times=np.unique(np.rint(np.r_[np.arange(0,tk[-1],.02),tk]*1e9).astype(np.int64))/1e9
                factor=max(float(np.max(abs(curve(times,nu=1))/(robot.velocity*scale))),float(np.sqrt(np.max(abs(curve(times,nu=2))/(robot.acceleration*scale**2)))))
                if factor<=1.000001:break
                dur=np.ceil(dur*max(1.02,factor*1.01)/.02)*.02
            else:continue
            path=curve(times)
            limits=robot.limits[14:21]
            if np.min(np.minimum(q[14:21]-limits[:,0],limits[:,1]-q[14:21]))<.03:continue
            endpoint_margin=np.minimum(np.minimum(contact[14:21]-limits[:,0],limits[:,1]-contact[14:21]),np.minimum(scan[14:21]-limits[:,0],limits[:,1]-scan[14:21]))
            required_margin=np.minimum(.03,endpoint_margin)
            if np.any(np.minimum(path[:,14:21]-limits[:,0],limits[:,1]-path[:,14:21])<required_margin-1e-5):continue
            points=np.array([robot.ee_fk('right',row)[:3,3] for row in path]);wi=int(np.argmin(abs(times-tk[1])))
            first=points[:wi+1];second=points[wi:];crossing=second[second[:,1]>initial[1,3]+.05]
            if np.min(first[:,2])<contact_pose[2,3]-.002 or np.max(first[:,1])>contact_pose[1,3]+.005:continue
            if len(crossing) and np.min(crossing[:,2])<1.21:continue
            if np.min(second[:,2])<data['xyz'][2]-.004:continue
            if np.min(second[:,1])<data['xyz'][1]-.012:continue
            if np.max(points[:,2])>final[2,3]+.004:continue
            valid_direct.append((float(times[-1])+data['orientation_blend']*.4,dict(data,t=times,path=path,ee=points,way_index=wi,initial_velocity_rad_s=contact_v.tolist())))
        for _,data in sorted(valid_direct,key=lambda x:x[0]):
            try:
                margin=validate_conveyor_path(robot,data['path']);chosen=data;chosen['conveyor_clearance_m']=margin;break
            except ValueError:continue
        if chosen is not None:break
    if chosen is None:raise RuntimeError('no checked contact/lift -> high -> scan proposal')
    d=chosen;prefix_t=np.linspace(3.,join_at,int(np.ceil((join_at-3.)/.02))+1)
    prefix=np.tile(start,(len(prefix_t),1));prefix[:,14:21]=lib.curves[key](prefix_t)
    path=np.vstack([prefix,d['path'][1:]]);times=np.r_[prefix_t-3.,join_at-3.+d['t'][1:]]
    ee=np.array([robot.ee_fk('right',row)[:3,3] for row in path]);way_index=len(prefix_t)-1+d['way_index']
    full_margin=validate_conveyor_path(robot,path)
    old_t,old_path=hermite([start,scan],robot.velocity*.5,robot.acceleration*.5**2,.02)
    old_lift=np.tile(start,(101,1));old_lift[:,14:21]=lib.curves[key](np.linspace(3.,5.,101))
    old_path=np.vstack([old_lift,old_path[1:]]);old_ee=np.array([robot.ee_fk('right',q)[:3,3] for q in old_path])
    frames=[]
    frame_indices=sorted(set(list(range(0,len(times),3))+[len(times)-1]))
    for idx in frame_indices:
        t,q=times[idx],path[idx]
        fk=robot.fk(q);arms={}
        for side,prefix in [('left','arm_l_joint'),('right','arm_r_joint')]:
            pts=[fk[robot.joints[prefix+str(i)]['child']][:3,3].tolist() for i in range(1,8)]
            pts.append(fk[robot.ee[side]][:3,3].tolist());arms[side]=pts
        frames.append(dict(t=float(t),right_R=robot.ee_fk('right',q)[:3,:3].tolist(),**arms))
    from deux_task_core.conveyor_screen import surfaces
    polygons=[vertices.tolist() for vertices,_,_ in surfaces()[3]]
    result=dict(schema='deux.offline.right_high_scan.v1',frame='base_link',robot_commands=False,
        source_cell=key,grasp_xyz_m=ee[0].tolist(),lifted_stop_xyz_m=initial[:3,3].tolist(),
        waypoint_xyz_m=robot.ee_fk('right',d['q'])[:3,3].tolist(),waypoint_joints_rad=d['q'][14:21].tolist(),
        scan_xyz_m=final[:3,3].tolist(),waypoint_orientation_blend=d['orientation_blend'],
        original_transfer_duration_s=float(old_t[-1]+2.),original_reference_speed_scale=.5,previous_high_proposal_duration_s=6.95,new_transfer_duration_s=float(times[-1]),join_original_trajectory_at_s=join_at,original_lift_prefix_s=join_at-3.,initial_velocity_rad_s=d['initial_velocity_rad_s'],
        waypoint_at_s=float(times[way_index]),minimum_conveyor_clearance_m=full_margin,
        first_leg_min_z_m=float(np.min(d['ee'][:d['way_index']+1,2])),
        first_leg_max_y_m=float(np.max(d['ee'][:d['way_index']+1,1])),
        minimum_z_after_moving_toward_scan_m=float(np.min(d['ee'][d['way_index']:][d['ee'][d['way_index']:,1]>initial[1,3]+.05,2])),
        minimum_right_joint_limit_margin_rad=float(np.min(np.minimum(path[:,14:21]-robot.limits[14:21,0],robot.limits[14:21,1]-path[:,14:21]))),
        maximum_grasp_z_m=float(np.max(ee[:,2])),scan_height_overshoot_m=max(0.,float(np.max(ee[:,2])-final[2,3])),curve_type='C2 quintic; non-stop waypoint; original velocity/acceleration at join',joint_speed_scale=scale,search_s=time.monotonic()-began,ik_candidates=len(candidates),pose_filtered_candidates=len(valid_direct),
        limitations=['incoming bag height/size/deformation not measured; dynamic bag collision avoidance not certified','one real-library normal-angle contact cell preview; other cells and +/-90deg paths need separate transition checks','contact at original 3s unchanged; a short original lift prefix is retained, then smooth higher/right exit replaces remaining low lift/brake; no deployment or robot motion'],
        times_s=times.tolist(),positions_rad=path.tolist(),ee_path_m=ee.tolist(),old_ee_path_m=old_ee.tolist(),frames=frames,conveyor_polygons=polygons)
    (out/'proposal.json').write_text(json.dumps(result,indent=2)+'\n');(out/'search-rejections.json').write_text(json.dumps(rows,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['times_s','positions_rad','ee_path_m','old_ee_path_m','frames','conveyor_polygons']},indent=2))


if __name__=='__main__':main()
