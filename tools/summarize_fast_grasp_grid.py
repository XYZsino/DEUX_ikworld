"""Check complete fast library, select a filled rectangle, and plot contact grid."""
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/deux-grid-matplotlib')
import json
from collections import Counter
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from scipy.interpolate import CubicHermiteSpline
from build_fast_grasp_grid import ROOT, digest, FastBuilder, V


def main():
    root = ROOT/'artifacts/right_grasp_grid_fast'
    m = json.loads((root/'manifest.json').read_text())
    a, r = m['accepted'], m['rejected']
    bounds = np.rint(np.array(m['search_xy_bounds_m'])/.03).astype(int)
    expected = {(x, y) for x in range(bounds[0, 0], bounds[1, 0]+1) for y in range(bounds[0, 1], bounds[1, 1]+1)}
    records = [tuple(np.rint(np.array(p['xy_m'])/.03).astype(int)) for p in a+r]
    if len(records) != len(set(records)) or set(records) != expected: raise ValueError('incomplete/duplicate grid')
    for path, sha in m['inputs'].items():
        if digest(ROOT/path) != sha: raise ValueError('source hash mismatch')
    builder = FastBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    max_close_relative_speed = 0.
    for p in a:
        if digest(root/p['file']) != p['sha256']: raise ValueError('trajectory hash mismatch')
        plan = json.loads((root/p['file']).read_text())
        curve = CubicHermiteSpline(plan['times_s'], plan['positions_rad'], plan['velocities_rad_s'])
        schedule = plan['hand_schedule']; events = plan['events']
        if (schedule['close_start_s']+schedule['close_duration_s'] != events['contact_s']
                or events['close_complete_s'] != events['contact_s']
                or events['lift_start_s'] != events['contact_s']):
            raise ValueError('closure is not synchronized with contact')
        for t in np.linspace(schedule['close_start_s'], events['contact_s'], 51):
            _, jac = builder.fk_jac(curve(t))
            max_close_relative_speed = max(max_close_relative_speed, float(np.linalg.norm((jac[:3]@curve(t, 1)-V)[:2])))
    if max_close_relative_speed > .02: raise ValueError('close relative XY velocity >2cm/s')
    summary = dict(valid_cells=len(a), rejected_cells=len(r), contact_s=3., close_start_s=2.5, close_complete_s=3.,
                   lift_start_s=3., lift_complete_s=4.5, stopped_s=5., velocity_m_s=[0., -.09, 0.], orientation_cone_deg=30.,
                   complete_grid_and_hashes_verified=True, max_close_relative_xy_speed_m_s=max_close_relative_speed,
                   parallel_hand_helper_sha256=digest(ROOT/'tools/parallel_grid_hand.py'),
                   rejection_counts=dict(Counter(p['reason'] for p in r)))
    fig, ax = plt.subplots(figsize=(10, 8))
    if r:
        rejected = np.array([p['xy_m'] for p in r]); ax.scatter(*rejected.T, s=4, c='#dddddd', label='Solver/path rejected')
    if a:
        xy = np.array([p['xy_m'] for p in a]); cells = {tuple(np.rint(p/.03).astype(int)) for p in xy}
        xs = range(min(x for x, y in cells), max(x for x, y in cells)+1)
        ys = range(min(y for x, y in cells), max(y for x, y in cells)+1)
        best = (0, None)
        for left in xs:
            common = set(ys)
            for right in range(left, xs.stop):
                common &= {y for x, y in cells if x == right}
                run = []
                for y in list(ys)+[ys.stop]:
                    if y in common: run.append(y)
                    elif run:
                        size = (right-left+1)*len(run)
                        if size > best[0]: best = size, [left, right, run[0], run[-1]]
                        run = []
        count, (xmin, xmax, ymin, ymax) = best
        rectangle = dict(x_min_m=round(xmin*.03, 2), x_max_m=round(xmax*.03, 2),
                         y_min_m=round(ymin*.03, 2), y_max_m=round(ymax*.03, 2), cells=count)
        selected = [p for p in a if xmin-1e-6 <= p['xy_m'][0]/.03 <= xmax+1e-6 and ymin-1e-6 <= p['xy_m'][1]/.03 <= ymax+1e-6]
        assert len(selected) == count
        (root/'recommended_cells.json').write_text(json.dumps(dict(frame='base_link', contact_z_m=.97, rectangle=rectangle, cells=selected), indent=2))
        summary.update(recommended_full_rectangle=rectangle, valid_xy_bounds_m=[xy.min(axis=0).tolist(), xy.max(axis=0).tolist()],
                       max_position_error_m=max(p['checks']['position_error_m'] for p in a),
                       max_orientation_error_deg=max(p['checks']['orientation_error_deg'] for p in a),
                       max_velocity_limit_ratio=max(p['checks']['velocity_limit_ratio'] for p in a),
                       max_acceleration_limit_ratio=max(p['checks']['acceleration_limit_ratio'] for p in a))
        ax.scatter(*xy.T, s=13, color='#147d92', label='Complete moving trajectory passed')
        ax.add_patch(Rectangle((xmin*.03-.015, ymin*.03-.015), (xmax-xmin+1)*.03, (ymax-ymin+1)*.03,
                               fill=False, edgecolor='#e46b19', linewidth=2, label=f'Filled rectangle: {count} cells'))
    ax.annotate('Belt: -Y\n0.09 m/s', xy=(.66, -.09), xytext=(.66, .17), arrowprops=dict(arrowstyle='->'), ha='center')
    if a: ax.set(xlim=(.25, .72), ylim=(-.18, .28))
    ax.set(xlabel='base_link X (m)', ylabel='base_link Y (m)', aspect='equal',
           title=f'Moving grasp: contact at 3.0 s, Z=0.97 m, orientation cone 30 deg\n{len(a)} complete paths; offline model evidence only')
    ax.grid(alpha=.2); ax.legend(fontsize=8); fig.tight_layout(); fig.savefig(root/'workspace.png', dpi=160)
    (root/'summary.json').write_text(json.dumps(summary, indent=2))
    (root/'README.rst').write_text('''Moving right-arm grasp library
==============================

Offline URDF evidence only; no hardware commands or physical grasp verification.
Original slow library is preserved in artifacts/right_grasp_grid/.
Previous sequential-close fast library is backed up in
.setup/right-grasp-fast-before-parallel/.

Timing and selection
--------------------

Each 3cm cell represents the ACTUAL GRASP POINT AT CONTACT (t=3.0 s).
All paths start from installation.yaml home_joints, identical stationary ready.
Object motion: base_link (0,-0.09,0) m/s, no object rotation.
At dispatch the grasp point must be 27cm upstream (+Y) of the selected cell.
Approach allocation is saved per trajectory; continuous tracking/descent ends
at t=3.0 s. Close runs IN PARALLEL from 2.5 to 3.0 s, finishing at contact.
Lift starts at contact, ends at 4.5 s; braking stops the arm at 5.0 s.
Upper height is 8cm; lift is 10cm. Close overlaps the final 0.5s of descent
and follows the bag by 4.5cm in -Y, while Z is still decreasing.
During lift the held object follows belt velocity; after clearing it, braking
smoothly brings the arm to rest. Final XY is contact XY plus (0,-0.1575)m.

Use the existing timestamped prediction topic, convert object origin to grasp
origin, compensate acquisition age, and compute ETA to a candidate contact cell.
For -Y motion, ETA=(current_grasp_y-cell_y)/0.09. Dispatch when ETA=3.0s,
allowing actual controller start latency in the scheduled start timestamp.
Require candidate lateral X error and prediction uncertainty to fit the actual
grasp error budget; nearest-grid quantization can introduce 2.12cm error.
Do not claim the 3mm MODEL path residual is a validated live object tolerance.
If too late, no valid path, object ID/direction/speed/orientation changes,
stale observations or missed tracking, refuse/abort through the guarded runtime.
Continue monitoring latest predictions and arm/hand feedback during execution.

Playback
--------

Use only accepted manifest entries, verify source and trajectory SHA256.
JSON contains the seven named right-arm joints, times_s, positions_rad and
velocities_rad_s. Preserve cubic Hermite interpolation and all timing; positions
alone do not reproduce the validated path. Do not stop at stage boundaries:
the arm is moving at contact, throughout Close and into Lift.
Use the existing checked RightTrialHand adapter, ready fraction before start,
fraction=1 repeated every 20ms from 2.5s through 3.0s, then remain closed.
Schedule against the same actual ROS trajectory start timestamp as the arm.
Launch the hand scheduling task concurrently with the arm action; never await
arm completion or contact before starting closure. If command timing is missed,
abort rather than shifting closure after contact. Existing late-height closure
gates must not silently delay this library schedule.
tools/parallel_grid_hand.py provides close_during_descent(), a coroutine for a
future grid replay caller to run concurrently with the arm. It takes the same
scheduled start timestamp, the existing checked hand adapter, clock/pause,
goal/state guard and live-feedback qualification callbacks. It refuses a missed
start (>20ms), a command scheduling gap, clock reversal or unqualified feedback.
Its result records command timing only, with physical_closure_verified=false.
The helper is tested with a virtual clock but is not wired into an active node.
Hand driver timing and physical closure/contact require hardware verification.
Runtime feedback checks must gate lift/abort on missed closure; the nominal
offline schedule does not certify the hand will complete by 3.0s. The 0.5s
duration comes from installation.yaml; measured driver/transport delay and
physical closure duration are not newly measured or compensated by this change.
This generator does not modify, connect or start the existing ROS 2 Jazzy executor.

Validation and limitations
--------------------------

Independent URDF FK: <=3mm sampled position error, <=30deg total SO(3) angular
distance from the original reference orientation from descent start to stop.
This is an orientation cone, not three independent Euler angle allowances.
Ready/approach can initially be outside this cone.
Exact continuous joint polynomial extrema: limits and <=90% configured velocity
and acceleration. This is kinematic feasibility, not torque/payload certification.
Existing collision spheres plus measured visible conveyor capsules, sampled
every 10ms. Hidden structure, full body/self collision and held-object volume
remain unverified. Solver failures are not proofs of physical unreachability.
No online trajectory selection/dispatch integration or hardware test was performed.

Reproduce
---------

  /usr/bin/python3 tools/build_fast_grasp_grid.py
  /usr/bin/python3 tools/test_fast_grasp_grid.py
  /usr/bin/python3 tools/test_parallel_grid_hand.py
  /usr/bin/python3 tools/summarize_fast_grasp_grid.py
''')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__': main()
