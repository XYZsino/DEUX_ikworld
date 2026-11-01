"""Offline axis-preserving +/-90deg bag library; no ROS or hardware commands."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import argparse,json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor,as_completed
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicHermiteSpline
from explore_grasp_rotations import FixedRotationBuilder,ROOT,target_position,digest

HEADINGS=list(range(-120,-59,5))


class TransverseBuilder(FixedRotationBuilder):
    def set_heading(self,heading):
        self.base_reference=np.asarray(self.config['grasp_object_from_right_ee'])[:3,:3]
        self.heading=float(heading)
        self.target_rotation=Rotation.from_euler('z',heading,degrees=True).as_matrix()@self.base_reference
        # The operator explicitly permits a new orientation for transverse bags.
        # Keep the inherited hard cone relative to THIS target, and independently
        # enforce the much tighter fixed-target/axis residual over the whole path.
        self.rotation=self.target_rotation.copy()

    def transverse_plan(self,point,approach):
        p=self.fixed_plan(point,approach)
        s=CubicHermiteSpline(p['times_s'],p['positions_rad'],p['velocities_rad_s'])
        max_axis=max_base=0.
        for t in np.r_[np.arange(approach,5.,.01),3.,5.]:
            q=self.ready.copy();q[14:21]=s(t);r=self.robot.ee_fk('right',q)[:3,:3]@self.base_reference.T
            actual=np.degrees(np.arctan2(r[1,0],r[0,0]))
            max_axis=max(max_axis,abs((actual-self.heading+90.)%180.-90.))
            max_base=max(max_base,float(np.degrees(Rotation.from_matrix(r).magnitude())))
        if max_axis>.5:raise ValueError('constant transverse axis residual >0.5deg')
        p['checks'].update(orientation_error_deg=max_base,axis_heading_error_deg=max_axis)
        p.update(selection_class='transverse',axis_heading_deg=self.heading,
                 front_back_reversal_allowed=True,
                 rotation_search=dict(id=f'transverse_yaw{int(self.heading):+04d}',yaw_deg=self.heading),
                 orientation_policy='operator-authorized transverse exception; fixed target, 180deg axis symmetry')
        return p


def worker(job):
    ix,iy,out=job;point=[round(ix*.03,8),round(iy*.03,8),.97]
    b=TransverseBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml');records=[]
    for heading in HEADINGS:
        b.rng=np.random.default_rng(970);b.set_heading(heading)
        row=dict(xy_m=point[:2],axis_heading_deg=heading)
        try:
            contact=b.relaxed(np.array(point),b.ready,attempts=4)
            b.relaxed(target_position(point,5.,1.5),contact,attempts=4)
        except ValueError as error:
            records.append(dict(row,accepted=False,reason='endpoint: '+str(error)));continue
        errors=[]
        for approach in [1.5,1.8,2.,1.2]:
            try:
                p=b.transverse_plan(point,approach)
                name=f'trajectories/x{ix:+04d}_y{iy:+04d}_yaw{heading:+04d}.json'
                path=Path(out)/name;path.write_text(json.dumps(p,separators=(',',':')))
                row.update(accepted=True,file=name,sha256=digest(path),checks=p['checks'],events=p['events']);break
            except ValueError as error:errors.append(str(error))
        else:row.update(accepted=False,reason=errors[-1],approach_failures=errors)
        records.append(row)
    return records


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=12)
    parser.add_argument('--output',type=Path,default=ROOT/'artifacts/right_grasp_transverse_search')
    args=parser.parse_args();out=args.output;(out/'trajectories').mkdir(parents=True,exist_ok=True)
    b=TransverseBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    sources=[ROOT/'src/deux_task_bringup/config/installation.yaml',Path(b.config['robot']['urdf_path']),
       ROOT/'calibration/conveyor_visible_surface.json',Path(__file__).resolve(),
       ROOT/'tools/explore_grasp_rotations.py',ROOT/'tools/build_fast_grasp_grid.py',
       ROOT/'tools/build_right_grasp_grid.py',ROOT/'tools/upper_collision_batch.py',
       ROOT/'src/deux_task_core/deux_task_core/model.py',ROOT/'src/deux_task_core/deux_task_core/conveyor_screen.py']
    m=dict(schema='deux.transverse_grasp_sweep.v1',complete=False,frame='base_link',
       ready_joints_28_rad=b.ready.tolist(),reference_rotation=b.rotation.tolist(),
       grid_step_m=.03,grasp_z_m=.97,velocity_m_s=[0.,-.09,0.],contact_s=3.,
       x_indices=[10,24],y_indices=[-6,5],canonical_headings_deg=HEADINGS,
       observed_yaw_intervals_deg=[[-120.,-60.],[60.,120.]],axis_match_tolerance_deg=3.,
       front_back_reversal_allowed=True,transverse_exempt_from_original_30deg_cone=True,
       inputs={str(p.relative_to(ROOT)):digest(p) for p in sources},records=[],
       limitations=['Offline existing collision model only; full body/self/held-object geometry unverified.',
        'Static left is installation home; actual fixed left must be separately audited.',
        'Finite deterministic IK/path search; failures are not mathematical unreachability proofs.',
        'No hardware commands. Same ready, contact 3s, close 2.5..3s, lift 4.5s, stop 5s.',
        'Fixed angle from descent start through lifted stop; ready-to-upper reorientation is included.'])
    jobs=[(x,y,str(out)) for x in range(10,25) for y in range(-6,6)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(worker,j) for j in jobs]
        for number,future in enumerate(as_completed(futures),1):
            m['records'].extend(future.result())
            if number%10==0 or number==len(jobs):
                (out/'manifest.json').write_text(json.dumps(m,indent=2))
                print(json.dumps(dict(cells_done=number,cells_total=len(jobs),accepted=sum(r['accepted'] for r in m['records']))),flush=True)
    m['complete']=True;m['records'].sort(key=lambda r:(r['xy_m'],r['axis_heading_deg']))
    (out/'manifest.json').write_text(json.dumps(m,indent=2))


if __name__=='__main__':main()
