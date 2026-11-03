"""Independent saved-path FK, analytic joint extrema, coverage, and pose report."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ.setdefault('MPLCONFIGDIR','/tmp/deux-grid-matplotlib')
import json
from collections import Counter
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicHermiteSpline
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build_transverse_grasp_grid import ROOT,HEADINGS,TransverseBuilder,target_position,digest


def main():
    out=ROOT/'artifacts/right_grasp_transverse_search';m=json.loads((out/'manifest.json').read_text())
    if not m['complete']:raise ValueError('incomplete sweep')
    for source,sha in m['inputs'].items():
        if digest(ROOT/source)!=sha:raise ValueError('changed source '+source)
    expected={(x,y,h) for x in range(10,25) for y in range(-6,6) for h in HEADINGS}
    actual=[(round(r['xy_m'][0]/.03),round(r['xy_m'][1]/.03),r['axis_heading_deg']) for r in m['records']]
    if len(actual)!=len(set(actual)) or set(actual)!=expected:raise ValueError('coverage/duplicate')
    b=TransverseBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    accepted=[r for r in m['records'] if r['accepted']];peaks=dict(position_m=0.,fixed_rotation_deg=0.,axis_deg=0.,velocity_ratio=0.,acceleration_ratio=0.)
    for index,row in enumerate(accepted,1):
        path=out/row['file']
        if digest(path)!=row['sha256']:raise ValueError('changed trajectory')
        p=json.loads(path.read_text());times=np.array(p['times_s']);positions=np.array(p['positions_rad']);velocities=np.array(p['velocities_rad_s'])
        s=CubicHermiteSpline(times,positions,velocities)
        np.testing.assert_allclose(s(0),b.ready[14:21],atol=1e-12)
        np.testing.assert_allclose(s(0,1),0.,atol=1e-12);np.testing.assert_allclose(s(5.,1),0.,atol=1e-10)
        e=p['events'];assert e['contact_s']==e['close_complete_s']==e['lift_start_s']==3.
        assert e['close_start_s']==2.5 and e['lift_complete_s']==4.5 and e['stopped_s']==5.
        for j in range(7):
            curve=CubicHermiteSpline(times,positions[:,j],velocities[:,j])
            for order in (0,1,2):
                scalar=curve.derivative(order);roots=scalar.derivative().roots(extrapolate=False)
                values=scalar(np.r_[times,np.nextafter(times[1:],times[:-1]),roots[np.isfinite(roots)]])
                if order==0:
                    if values.min()<b.lo[j] or values.max()>b.hi[j]:raise ValueError('analytic joint bound')
                else:
                    limits=b.robot.velocity if order==1 else b.robot.acceleration
                    ratio=float(np.max(abs(values))/limits[j+14]);name='velocity_ratio' if order==1 else 'acceleration_ratio'
                    peaks[name]=max(peaks[name],ratio)
                    if ratio>.90000001:raise ValueError('analytic derivative bound')
        target=np.asarray(p['target_rotation_base']);reference=np.asarray(m['reference_rotation'])
        approach=e['approach_complete_s']
        for t in np.r_[np.arange(approach+.005,5.,.01),3.,5.]:
            q=b.ready.copy();q[14:21]=s(t);pose=b.robot.ee_fk('right',q)
            position=float(np.linalg.norm(pose[:3,3]-target_position(p['contact_xyz_m'],t,approach)))
            angle=float(np.degrees(Rotation.from_matrix(pose[:3,:3]@target.T).magnitude()))
            relative=pose[:3,:3]@reference.T;heading=np.degrees(np.arctan2(relative[1,0],relative[0,0]))
            axis=abs((heading-p['axis_heading_deg']+90.)%180.-90.)
            if position>.003 or angle>1. or axis>.5:raise ValueError('independent fixed-axis FK check')
            peaks['position_m']=max(peaks['position_m'],position);peaks['fixed_rotation_deg']=max(peaks['fixed_rotation_deg'],angle);peaks['axis_deg']=max(peaks['axis_deg'],float(axis))
        if index%100==0:print(json.dumps(dict(verified=index,total=len(accepted))),flush=True)
    by_x=[];representatives=[]
    for ix in range(10,25):
        rows=[r for r in accepted if round(r['xy_m'][0]/.03)==ix]
        ninety=[r for r in rows if r['axis_heading_deg']==-90]
        headings=sorted({r['axis_heading_deg'] for r in rows})
        entry=dict(x_m=round(ix*.03,2),paths=len(rows),xy_cells=len({tuple(r['xy_m']) for r in rows}),
                   supported_canonical_headings_deg=headings,missing_canonical_headings_deg=sorted(set(HEADINGS)-set(headings)),exact_90_paths=len(ninety))
        if ninety:
            best=min(ninety,key=lambda r:r['checks']['acceleration_limit_ratio']);p=json.loads((out/best['file']).read_text())
            s=CubicHermiteSpline(p['times_s'],p['positions_rad'],p['velocities_rad_s'])
            pose=dict(x_m=entry['x_m'],contact_xyz_m=p['contact_xyz_m'],axis_heading_deg=-90.,
                      target_rotation_base=p['target_rotation_base'],source_file=best['file'],sha256=best['sha256'],
                      contact_right_joints_rad=s(3.).tolist(),upper_right_joints_rad=s(p['events']['approach_complete_s']).tolist(),
                      stopped_right_joints_rad=s(5.).tolist(),checks=p['checks'])
            representatives.append(pose);entry['representative_file']=best['file']
        by_x.append(entry)
    counts=Counter(tuple(r['xy_m']) for r in accepted)
    summary=dict(complete=True,independent_all_paths_verified=True,combinations=len(expected),
                 accepted_paths=len(accepted),contact_cells=len(counts),by_x=by_x,peaks=peaks,
                 rejection_counts=dict(Counter(r['reason'] for r in m['records'] if not r['accepted'])),
                 evidence='offline actual URDF and existing measured-surface model; no hardware execution',
                 fixed_axis_from='upper approach completion through contact/lift/stopped',
                 observed_yaw_intervals_deg=m['observed_yaw_intervals_deg'],front_back_reversal_allowed=True)
    (out/'summary.json').write_text(json.dumps(summary,indent=2));(out/'poses_by_x.json').write_text(json.dumps(representatives,indent=2))
    matrix=np.array([[sum(round(r['xy_m'][0]/.03)==ix and r['axis_heading_deg']==h for r in accepted) for ix in range(10,25)] for h in HEADINGS])
    fig,ax=plt.subplots(figsize=(12,6));im=ax.imshow(matrix,origin='lower',aspect='auto',cmap='viridis',vmin=0)
    ax.set_xticks(range(15),[f'{ix*.03:.2f}' for ix in range(10,25)]);ax.set_yticks(range(len(HEADINGS)),HEADINGS)
    ax.set(xlabel='Contact X in base_link (m)',ylabel='Stored canonical yaw (deg; +90/-90 share the same axis)',
           title=f'Transverse bag: contact 3s, fixed angle, Z=0.97m\n{len(accepted)} paths; offline model verification')
    fig.colorbar(im,ax=ax,label='Number of verified contact Y cells');fig.tight_layout();fig.savefig(out/'x_angle_coverage.png',dpi=160)
    print(json.dumps(dict(paths=len(accepted),cells=len(counts),exact_90_x_m=[p['x_m'] for p in representatives],peaks=peaks)),flush=True)


if __name__=='__main__':main()
