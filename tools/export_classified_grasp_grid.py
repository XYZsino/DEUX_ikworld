"""Combine unchanged normal library with verified transverse-axis paths."""
import json,shutil
from pathlib import Path
from build_right_grasp_grid import ROOT,digest


def main():
    normal=ROOT/'artifacts/right_grasp_grid_rotations';special=ROOT/'artifacts/right_grasp_transverse_search'
    n=json.loads((normal/'manifest.json').read_text());t=json.loads((special/'manifest.json').read_text())
    summary=json.loads((special/'summary.json').read_text())
    if not summary['independent_all_paths_verified'] or not t['complete']:raise ValueError('verified transverse sweep required')
    if n['ready_joints_28_rad']!=t['ready_joints_28_rad'] or n['velocity_m_s']!=t['velocity_m_s']:raise ValueError('ready/timing source mismatch')
    inputs=dict(n['inputs'])
    for name,sha in t['inputs'].items():
        if name in inputs and inputs[name]!=sha:raise ValueError('different source versions: '+name)
        inputs[name]=sha
    for name,sha in inputs.items():
        if digest(ROOT/name)!=sha:raise ValueError('changed model/source '+name)
    out=ROOT/'artifacts/right_grasp_grid_classified';out.mkdir(parents=True,exist_ok=True);accepted=[]
    normal_recommended={p['file'] for p in json.loads((normal/'recommended_cells.json').read_text())['cells']}
    recommended=[]
    for source,items,prefix in [(normal,n['accepted'],'normal'),(special,[r for r in t['records'] if r['accepted']],'transverse')]:
        for item in items:
            path=source/item['file']
            if digest(path)!=item['sha256']:raise ValueError('changed trajectory')
            name=prefix+'/'+item['file'];target=out/name;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,target)
            row={k:item[k] for k in ['xy_m','sha256','checks']};row.update(file=name,selection_class=prefix)
            accepted.append(row)
            if prefix=='transverse' or item['file'] in normal_recommended:recommended.append(row)
    for path in [normal/'manifest.json',special/'manifest.json',special/'summary.json',Path(__file__).resolve(),ROOT/'tools/classified_grid_runtime.py']:
        inputs[str(path.relative_to(ROOT))]=digest(path)
    m=dict(schema='deux.fast_grasp_grid.v2',frame='base_link',grid_step_m=.03,grasp_z_m=.97,
        velocity_m_s=n['velocity_m_s'],ready_joints_28_rad=n['ready_joints_28_rad'],reference_rotation=n['reference_rotation'],
        inputs=inputs,accepted=accepted,orientation_policy='normal <=30deg; transverse operator exception with fixed target <=1deg',
        transverse_policy=dict(observed_yaw_intervals_deg=[[-120,-60],[60,120]],front_back_reversal_allowed=True,
                               axis_match_tolerance_deg=3.,no_normal_fallback=True),
        limitations=t['limitations'])
    (out/'manifest.json').write_text(json.dumps(m,indent=2))
    (out/'recommended_cells.json').write_text(json.dumps(dict(cells=recommended,
       policy='normal: original 80-cell rectangle; transverse: retain all verified angle-specific paths'),indent=2))
    print(json.dumps(dict(directory=str(out),normal_paths=len(n['accepted']),transverse_paths=summary['accepted_paths'],total_paths=len(accepted))))


if __name__=='__main__':main()
