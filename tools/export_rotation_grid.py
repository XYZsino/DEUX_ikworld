"""Convert verified research paths to the stored runtime v2 format; no ROS."""
import json
import shutil
from pathlib import Path
from build_right_grasp_grid import ROOT, digest


def main():
    source = ROOT/'artifacts/right_grasp_rotation_search'
    out = ROOT/'artifacts/right_grasp_grid_rotations'; out.mkdir(exist_ok=True, parents=True)
    m = json.loads((source/'manifest.json').read_text())
    s = json.loads((source/'summary.json').read_text())
    if not m['complete'] or not s['complete_grid_and_source_and_trajectory_hashes_verified']:
        raise ValueError('verified complete exploration required')
    for name, sha in m['inputs'].items():
        if digest(ROOT/name) != sha: raise ValueError('input changed: '+name)
    accepted = []
    for item in m['records']:
        if not item['accepted']: continue
        path = source/item['file']
        if digest(path) != item['sha256']: raise ValueError('source trajectory changed')
        target = out/item['file']; target.parent.mkdir(exist_ok=True, parents=True)
        shutil.copy2(path, target)
        accepted.append({k:item[k] for k in ('xy_m','file','sha256','checks','rotation_id')})
    inputs = dict(m['inputs'])
    for path in [source/'manifest.json', Path(__file__).resolve(), ROOT/'tools/grid_grasp_runtime.py']:
        inputs[str(path.relative_to(ROOT))] = digest(path)
    runtime = dict(schema='deux.fast_grasp_grid.v2', frame='base_link', grid_step_m=.03,
                   grasp_z_m=.97, velocity_m_s=m['velocity_m_s'], contact_s=3., orientation_cone_deg=30.,
                   reference_rotation=m['reference_rotation'], ready_joints_28_rad=m['ready_joints_28_rad'],
                   inputs=inputs, accepted=accepted, source_schema=m['schema'],
                   selection='XY then ETA, yaw-only target rotation is a soft tie-break at the same cell',
                   limitations=m['limitations'], hand_schedule_note='nominal parallel commands; physical grasp unverified')
    (out/'manifest.json').write_text(json.dumps(runtime, indent=2))
    rect = s['fixed_rotation_full_rectangle']
    selected = [p for p in accepted if rect['x_min_m']-1e-9 <= p['xy_m'][0] <= rect['x_max_m']+1e-9
                and rect['y_min_m']-1e-9 <= p['xy_m'][1] <= rect['y_max_m']+1e-9]
    (out/'recommended_cells.json').write_text(json.dumps(dict(frame='base_link', contact_z_m=.97,
        rectangle=rect, cells=selected, trajectories=len(selected)), indent=2))
    print(json.dumps(dict(directory=str(out), paths=len(accepted), cells=s['feasible_cells'],
                          recommended_cells=rect['cells'], recommended_paths=len(selected))))


if __name__ == '__main__': main()
