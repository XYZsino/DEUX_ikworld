"""Verify saved rotation sweep and produce an offline comparison plot/report."""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/deux-grid-matplotlib')
os.environ['OPENBLAS_NUM_THREADS'] = '1'
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from explore_grasp_rotations import ROOT, FixedRotationBuilder, rotations, target_position, digest


def rectangle(cells):
    if not cells:
        return None
    indices = {(round(x/.03), round(y/.03)) for x, y in cells}
    xmin, xmax = min(x for x, y in indices), max(x for x, y in indices)
    ymin, ymax = min(y for x, y in indices), max(y for x, y in indices)
    best = (0, None)
    for left in range(xmin, xmax+1):
        common = set(range(ymin, ymax+1))
        for right in range(left, xmax+1):
            common &= {y for x, y in indices if x == right}
            run = []
            for y in range(ymin, ymax+2):
                if y in common:
                    run.append(y)
                elif run:
                    count = (right-left+1)*len(run)
                    if count > best[0]:
                        best = count, (left, right, run[0], run[-1])
                    run = []
    count, (left, right, bottom, top) = best
    return dict(cells=count, x_min_m=round(left*.03, 2), x_max_m=round(right*.03, 2),
                y_min_m=round(bottom*.03, 2), y_max_m=round(top*.03, 2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/right_grasp_rotation_search')
    args = parser.parse_args(); root = args.output
    m = json.loads((root/'manifest.json').read_text())
    if not m['complete']:
        raise ValueError('sweep incomplete')
    for source, sha in m['inputs'].items():
        if digest(ROOT/source) != sha:
            raise ValueError('changed input '+source)
    for part in m.get('merged_parts', []):
        if digest(root/part['snapshot']) != part['sha256']:
            raise ValueError('changed source sweep receipt')
    expected = {(x, y, r['id']) for x in range(m['x_indices'][0], m['x_indices'][1]+1)
                for y in range(m['y_indices'][0], m['y_indices'][1]+1) for r in rotations()}
    keys = [(round(r['xy_m'][0]/.03), round(r['xy_m'][1]/.03), r['rotation_id']) for r in m['records']]
    if len(keys) != len(set(keys)) or set(keys) != expected:
        raise ValueError('incomplete/duplicate grid-rotation combinations')
    b = FixedRotationBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    accepted = [r for r in m['records'] if r['accepted']]
    reference = np.array(m['reference_rotation'])
    peaks = dict(position_m=0., reference_angle_deg=0., target_angle_deg=0.,
                 velocity_ratio=0., acceleration_ratio=0.)
    for entry in accepted:
        path = root/entry['file']
        if digest(path) != entry['sha256']:
            raise ValueError('trajectory hash mismatch')
        p = json.loads(path.read_text())
        s = CubicHermiteSpline(p['times_s'], p['positions_rad'], p['velocities_rad_s'])
        np.testing.assert_allclose(s(0), b.ready[14:21], atol=1e-12)
        np.testing.assert_allclose(s(0, 1), 0., atol=1e-12)
        np.testing.assert_allclose(s(5., 1), 0., atol=1e-10)
        assert p['events']['close_start_s'] == 2.5
        assert p['events']['contact_s'] == p['events']['close_complete_s'] == p['events']['lift_start_s'] == 3.
        assert p['events']['lift_complete_s'] == 4.5 and p['events']['stopped_s'] == 5.
        # Offset sample times from the generator's 10ms grid.
        times = np.r_[np.arange(.005, 5., .01), 3., 5.]
        values = s(times)
        if np.any(values < b.lo-1e-10) or np.any(values > b.hi+1e-10):
            raise ValueError('joint position limit')
        for order, name, limits in [(1, 'velocity_ratio', b.robot.velocity[14:21]),
                                    (2, 'acceleration_ratio', b.robot.acceleration[14:21])]:
            ratio = float(np.max(abs(s(times, order))/limits))
            peaks[name] = max(peaks[name], ratio)
            if ratio > .900001:
                raise ValueError('sampled derivative limit')
        approach = p['events']['approach_complete_s']
        for t in times[times >= approach]:
            q = b.ready.copy(); q[14:21] = s(t)
            actual = b.robot.ee_fk('right', q)
            residual = np.linalg.norm(actual[:3, 3]-target_position(p['contact_xyz_m'], t, approach))
            angle = np.degrees(Rotation.from_matrix(actual[:3, :3]@reference.T).magnitude())
            target_angle = np.degrees(Rotation.from_matrix(actual[:3, :3]@np.array(p['target_rotation_base']).T).magnitude())
            if residual > .003 or angle > 30. or target_angle > 1.:
                raise ValueError('independent FK residual')
            peaks['position_m'] = max(peaks['position_m'], float(residual))
            peaks['reference_angle_deg'] = max(peaks['reference_angle_deg'], float(angle))
            peaks['target_angle_deg'] = max(peaks['target_angle_deg'], float(target_angle))
    counts = Counter(tuple(r['xy_m']) for r in accepted)
    old = json.loads((ROOT/'artifacts/right_grasp_grid_fast/manifest.json').read_text())
    old_cells = {tuple(r['xy_m']) for r in old['accepted']}
    cells = set(counts)
    per_rotation = []
    for rotation in rotations():
        entries = [r for r in accepted if r['rotation_id'] == rotation['id']]
        per_rotation.append(dict(rotation, accepted_paths=len(entries),
                                 newly_feasible_cells=sum(tuple(r['xy_m']) not in old_cells for r in entries),
                                 full_rectangle=rectangle({tuple(r['xy_m']) for r in entries})))
    best_single = max(per_rotation, key=lambda r:(r['full_rectangle'] or {}).get('cells', 0))
    summary = dict(combinations=len(expected), rotations=len(rotations()),
                   solver_cell_rotation_attempts=sum(len(r.get('search_variants', [None])) for r in m['records']),
                   cells_tested=len(expected)//len(rotations()),
                   accepted_paths=len(accepted), feasible_cells=len(cells),
                   previous_feasible_cells=len(old_cells),
                   new_cells=[list(p) for p in sorted(cells-old_cells)],
                   combined_feasible_cells=len(cells|old_cells),
                   fixed_rotation_full_rectangle=rectangle(cells),
                   combined_full_rectangle=rectangle(cells|old_cells),
                   best_single_rotation_rectangle=best_single,
                   rotation_count_histogram=dict(Counter(counts.values())),
                   previous_cells_without_fixed_rotation_solution=[list(p) for p in sorted(old_cells-cells)],
                   accepted_xy_bounds_m=([np.min(list(cells), axis=0).tolist(), np.max(list(cells), axis=0).tolist()] if cells else None),
                   rejection_counts=dict(Counter(r['reason'] for r in m['records'] if not r['accepted'])),
                   per_rotation=per_rotation, independent_offset_sample_peaks=peaks,
                   complete_grid_and_source_and_trajectory_hashes_verified=True,
                   evidence='offline kinematic and existing collision-model screening; no hardware commands')
    fig, ax = plt.subplots(figsize=(10, 7))
    ax.scatter(*np.array(sorted(old_cells)).T, facecolors='none', edgecolors='gray', s=85, label='Previous relaxed cone cells')
    if cells:
        xy = np.array(sorted(cells))
        dots = ax.scatter(*xy.T, c=[counts[tuple(p)] for p in xy], cmap='viridis', vmin=1, s=44)
        fig.colorbar(dots, ax=ax, label='Feasible fixed rotations per cell')
    if cells-old_cells:
        ax.scatter(*np.array(sorted(cells-old_cells)).T, marker='+', color='red', s=100, label='Additional cells')
    ax.set(xlabel='base_link X (m)', ylabel='base_link Y (m)', aspect='equal',
           xlim=(.285,.735), ylim=(-.195,.255),
           title=f'53 fixed rotations, contact 3s, grasp Z 0.97m\n{len(accepted)} paths / {len(cells)} cells; offline model evidence')
    ax.grid(alpha=.2); ax.legend(); fig.tight_layout(); fig.savefig(root/'rotation_workspace.png', dpi=160)
    (root/'summary.json').write_text(json.dumps(summary, indent=2))
    cell_entries = []
    for xy in sorted(cells):
        options = [r for r in accepted if tuple(r['xy_m']) == xy]
        options.sort(key=lambda r:(r['checks']['acceleration_limit_ratio'], r['checks']['velocity_limit_ratio']))
        cell_entries.append(dict(xy_m=list(xy), feasible_fixed_rotations=len(options),
                                 lowest_acceleration_path=options[0], options=options))
    (root/'cells.json').write_text(json.dumps(cell_entries, indent=2))
    lines = ['Rotation exploration (offline only)', '==================================', '',
             f"Tested {len(expected)} combinations: {summary['cells_tested']} contact cells x 53 rotations.",
             f"XY grid: X {m['x_indices'][0]*.03:.2f}..{m['x_indices'][1]*.03:.2f}m, "
             f"Y {m['y_indices'][0]*.03:.2f}..{m['y_indices'][1]*.03:.2f}m, 0.03m spacing; base_link Z=0.97m.",
             'Rotations: reference plus 26 signed axis/face/body-diagonal directions at 15 and 29 degrees.',
             'R_target = Exp(rotvec_base) @ R_reference. This is not independent Euler +/-30deg.',
             'Target fixed through descent/lift/braking (<=1deg sampled error); <=30deg original cone.',
             'Ready pose identical to installation.yaml. Contact 3s; close 2.5..3s; lift 3..4.5s; stop 5s.',
             'Speed fixed at base -Y 0.09m/s; upper clearance 8cm, lift 10cm.', '',
             f"Accepted {len(accepted)} paths across {len(cells)} cells; {len(cells-old_cells)} additional cells.",
             'Previous relaxed-cone paths are preserved and are not replaced by fixed rotations.',
             'manifest.json contains every attempted combination, failure stage, hashes and assumptions.',
             'summary.json contains per-rotation counts and independent FK/derivative verification.', '',
             'Limitations', '-----------', '', *m['limitations'], '',
             'No fresh hardware-left-arm audit, torque/payload check, hand closure confirmation or physical grasp test.',
             'Do not deploy this research schema to GridLibrary without explicit conversion and runtime validation.',
             'Failures indicate this deterministic solver/path search failed, not mathematical infeasibility.', '',
             m.get('search_algorithm_provenance', 'One generator version used for all cells.'), '',
             'Run a new sweep / verify', '------------------------', '',
             '  /usr/bin/python3 tools/explore_grasp_rotations.py',
             '  /usr/bin/python3 tools/summarize_grasp_rotations.py', '',
             'A new full run uses the current solver for every cell and may produce different candidate paths.',
             'Historical source is .setup/rotation-search-original-solver/explore_grasp_rotations.py.',
             'Per-part snapshots retain the source hashes for the exact retained paths.', '']
    (root/'README.rst').write_text('\n'.join(lines))
    print(json.dumps({k:v for k,v in summary.items() if k != 'per_rotation'}, indent=2))


if __name__ == '__main__':
    main()
