"""Summarize completed library; verify hashes and export its XY map."""
import json
import os
from pathlib import Path
from collections import Counter
import numpy as np
os.environ.setdefault('MPLCONFIGDIR', '/tmp/deux-grid-matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from build_right_grasp_grid import ROOT, digest


def main():
    root = ROOT/'artifacts/right_grasp_grid'
    m = json.loads((root/'manifest.json').read_text())
    accepted = m['accepted']; rejected = m['rejected']
    bounds = np.rint(np.array(m['search_xy_bounds_m'])/.03).astype(int)
    expected = {(x, y) for x in range(bounds[0, 0], bounds[1, 0]+1)
                for y in range(bounds[0, 1], bounds[1, 1]+1)}
    recorded = [tuple(np.rint(np.array(p['xy_m'])/.03).astype(int)) for p in accepted+rejected]
    if len(recorded) != len(set(recorded)) or set(recorded) != expected:
        raise ValueError('grid build incomplete or contains duplicate cells')
    installation = ROOT/'src/deux_task_bringup/config/installation.yaml'
    import yaml
    config = yaml.safe_load(installation.read_text())
    urdf = installation.parent/config['robot']['urdf_path']
    for path, key in [(installation, 'installation_sha256'), (urdf, 'urdf_sha256'),
                      (ROOT/'calibration/conveyor_visible_surface.json', 'measured_surface_sha256')]:
        if digest(path) != m[key]: raise ValueError('source hash mismatch: '+str(path))
    if not accepted:
        raise ValueError('no valid cells')
    for item in accepted:
        if digest(root/item['file']) != item['sha256']:
            raise ValueError('trajectory hash mismatch')
    xy = np.array([item['xy_m'] for item in accepted])
    cells = {tuple(np.rint(p/.03).astype(int)) for p in xy}
    # Largest completely populated axis-aligned rectangle, not just bounding box.
    xs = range(min(x for x, y in cells), max(x for x, y in cells)+1)
    ys = range(min(y for x, y in cells), max(y for x, y in cells)+1)
    best = (0, None)
    for left in xs:
        common = set(ys)
        for right in range(left, xs.stop):
            common &= {y for x, y in cells if x == right}
            runs = []; run = []
            for y in ys:
                if y in common: run.append(y)
                elif run: runs.append(run); run = []
            if run: runs.append(run)
            for run in runs:
                count = (right-left+1)*len(run)
                if count > best[0]: best = count, [left, right, run[0], run[-1]]
    count, (xmin, xmax, ymin, ymax) = best
    recommended = [p for p in accepted if xmin-1e-6 <= p['xy_m'][0]/.03 <= xmax+1e-6
                   and ymin-1e-6 <= p['xy_m'][1]/.03 <= ymax+1e-6]
    assert len(recommended) == count
    (root/'recommended_cells.json').write_text(json.dumps(dict(frame='base_link', grasp_z_m=.97,
        grid_step_m=.03, cells=recommended), indent=2))
    summary = dict(valid_cells=len(accepted), rejected_cells=len(rejected),
                   valid_xy_bounds_m=[xy.min(axis=0).tolist(), xy.max(axis=0).tolist()],
                   recommended_full_rectangle=dict(x_min_m=xmin*.03, x_max_m=xmax*.03,
                                                   y_min_m=ymin*.03, y_max_m=ymax*.03, cells=count),
                   duration_range_s=[min(p['duration_s'] for p in accepted), max(p['duration_s'] for p in accepted)],
                   rejection_counts=dict(Counter(p['reason'] for p in rejected)),
                   all_file_hashes_verified=True, verification=m['verification'])
    (root/'summary.json').write_text(json.dumps(summary, indent=2))
    fig, ax = plt.subplots(figsize=(9, 8))
    failed = np.array([p['xy_m'] for p in rejected])
    if len(failed): ax.scatter(failed[:, 0], failed[:, 1], s=5, c='#dddddd', label='Rejected by solver/path')
    ax.scatter(xy[:, 0], xy[:, 1], s=12, c='#147d92', label='Full trajectory passed model checks')
    from matplotlib.patches import Rectangle
    ax.add_patch(Rectangle((xmin*.03-.015, ymin*.03-.015), (xmax-xmin+1)*.03, (ymax-ymin+1)*.03,
                           fill=False, edgecolor='#e46b19', linewidth=2, label=f'Recommended rectangle: {count} cells'))
    ax.set(xlabel='base_link X (m)', ylabel='base_link Y (m)',
           title=f'Right grasp library: Z=0.97 m, 3 cm grid\n{len(accepted)} valid cells; offline model checks only', aspect='equal')
    ax.grid(alpha=.2); ax.legend(loc='upper left', fontsize=8); fig.tight_layout()
    fig.savefig(root/'workspace.png', dpi=160)
    (root/'README.rst').write_text(f'''Right-arm precomputed grasp library
==================================

Offline model evidence only; hardware has not been commanded or verified.

Inputs and selection
--------------------

installation.yaml and its measured-lift URDF are recorded by SHA256 in manifest.json.
Grid: base_link origin anchored, spacing 0.03 m. Actual grasp point Z=0.97 m.
Object axes assumed aligned with base_link; grasp rotation is the installation
grasp_object_from_right_ee rotation, stored explicitly in the manifest.
All {len(accepted)+len(rejected)} points inside the configured scene XY box were tested.
{len(accepted)} complete paths passed. Rejections are solver/path failures, not
mathematical proofs of unreachability. Bounding boxes can contain invalid cells.

Recommended fully populated rectangle: X {xmin*.03:.2f}..{xmax*.03:.2f} m,
Y {ymin*.03:.2f}..{ymax*.03:.2f} m, {count} cells. See workspace.png.

Playback contract
-----------------

Select only an accepted file listed in manifest.json and verify its SHA256.
Each file contains named right-arm joints and sequential stages Approach,
Descend, Close, Lift. Each stage has times_s, positions_rad, velocities_rad_s.
Use cubic Hermite interpolation (positions AND velocities), with local stage
times starting at zero. Boundaries stop. Do not smooth, retime, join different
cells, or recompute IK at playback without new validation.

Start from the exact 28-joint ready snapshot in manifest.json; the left arm
is assumed stationary there. Only seven right-arm joints are exported.
First set the existing RightTrialHand ready fraction {b_ready_fraction()}.
Approach ends at Z=1.05 m; Descend ends at Z=0.97 m. Hold the arm for the
Close stage, sending fraction=1 through the existing checked RightTrialHand
adapter at 20 ms intervals for 0.5 s. Require measured hand completion before
starting Lift to Z=1.07 m. Keep the hand closed during Lift.
Arm commands must use the existing guarded ROS 2 Jazzy action execution path,
including fresh joint state, start tolerance, tracking/error/estop checks,
controller ownership, stage completion and timeout checks. A JSON file is
not a replacement for these runtime checks. No hardware replay node was
started by these offline tools.

Each cell is an independent trial ending lifted. Return-to-ready, release,
and travel between grid cells are outside this library and are not implicit.

Validation scope
----------------

IK knot residual <=1 mm / 1 degree; interpolated vertical path <=2 mm / 2 degrees.
Cartesian knots <=5 mm apart; zero knot velocity; cubic joint velocity and
acceleration maxima analytically retimed to <=50% of URDF/config limits.
Joint positions remain inside limits by monotone cubic interpolation.
Existing grasp spheres and measured conveyor capsule screening are checked
every 10 ms. This is sampled temporal collision evidence, not continuous-time
full robot collision proof. Hidden conveyor geometry, complete body/self
collision, held-object shape and physical grasp accuracy remain unverified.
Configured grasp transform contains SIM-derived orientation and an estimated
measured-joint pinch midpoint; it is not a newly calibrated physical contact.
Close is a hardware hand command schedule, not a finger FK/contact simulation.

Reproduce
---------

  /usr/bin/python3 tools/build_right_grasp_grid.py
  /usr/bin/python3 tools/test_right_grasp_grid.py
  /usr/bin/python3 tools/summarize_right_grasp_grid.py

Python dependencies: NumPy, SciPy, PyYAML, Matplotlib. The generator explicitly
uses offline SciPy IK; installation.yaml's PyRoKi runtime choice is unchanged.
''')
    print(json.dumps(summary, indent=2))


def b_ready_fraction():
    import yaml
    return yaml.safe_load((ROOT/'src/deux_task_bringup/config/installation.yaml').read_text()).get('right_hand_ready_fraction', 0.)


if __name__ == '__main__': main()
