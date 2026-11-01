"""Offline fixed-rotation sweep. Never imports ROS or connects to hardware."""
import os
for key in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS'):
    os.environ[key] = '1'
import argparse
import copy
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
import itertools
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from build_fast_grasp_grid import FastBuilder, ROOT, target_position
from build_right_grasp_grid import digest


def rotations():
    result = [dict(id='reference', rotvec_deg=[0., 0., 0.])]
    for index, direction in enumerate(itertools.product((-1, 0, 1), repeat=3)):
        axis = np.array(direction, float)
        if not np.any(axis):
            continue
        axis /= np.linalg.norm(axis)
        for angle in (15., 29.):
            result.append(dict(id=f'axis{index:02d}_{int(angle):02d}',
                               rotvec_deg=(axis*angle).tolist()))
    return result


class FixedRotationBuilder(FastBuilder):
    def relaxed(self, xyz, seed, attempts=1):
        reference = self.rotation
        self.rotation = self.target_rotation
        try:
            return self.solve(xyz, seed, retries=attempts)
        finally:
            self.rotation = reference

    def plan_fast(self, point, approach=1.5):
        # The ready-to-upper Hermite segment is fully determined by its first
        # three tracking knots. Reject its EXACT polynomial extrema before
        # solving the remaining ~60 knots. This does not loosen any check.
        rng_state = copy.deepcopy(self.rng.bit_generator.state)
        contact = self.relaxed(np.array(point), self.ready, attempts=2)
        self.relaxed(target_position(point, 2.5, approach), contact)
        times = np.linspace(approach, 5., round((5.-approach)/.05)+1)
        q = self.relaxed(target_position(point, times[0], approach), contact, attempts=2)
        first = [q.copy()]
        for t in times[1:3]:
            q = self.relaxed(target_position(point, t, approach), q)
            first.append(q.copy())
        velocity = np.gradient(np.array(first), times[:3], axis=0, edge_order=2)[0]
        for j in range(14, 21):
            curve = CubicHermiteSpline([0., approach], [self.ready[j], first[0][j]], [0., velocity[j]])
            for order, limit in [(0, None), (1, self.robot.velocity[j]), (2, self.robot.acceleration[j])]:
                derivative = curve.derivative(order)
                roots = derivative.derivative().roots(extrapolate=False)
                values = derivative(np.r_[0., approach, roots[np.isfinite(roots)]])
                if order == 0:
                    if values.min() < self.lo[j-14] or values.max() > self.hi[j-14]:
                        raise ValueError('approach continuous joint position limit')
                elif np.max(abs(values))/limit > .9:
                    raise ValueError('approach continuous '+('velocity' if order == 1 else 'acceleration')+' >90% limit')
        self.rng.bit_generator.state = rng_state
        return super().plan_fast(point, approach)

    def fixed_plan(self, point, approach):
        plan = self.plan_fast(point, approach)
        curve = CubicHermiteSpline(plan['times_s'], plan['positions_rad'], plan['velocities_rad_s'])
        max_error = max_relative_speed = 0.
        contact_rotation = None
        for t in np.linspace(approach, 5., round((5.-approach)/.01)+1):
            q = self.ready.copy(); q[14:21] = curve(t)
            pose = self.robot.ee_fk('right', q)
            max_error = max(max_error, float(Rotation.from_matrix(
                pose[:3, :3]@self.target_rotation.T).magnitude()))
        for t in np.linspace(2.5, 3., 51):
            _, jac = self.fk_jac(curve(t))
            max_relative_speed = max(max_relative_speed, float(np.linalg.norm(
                (jac[:3]@curve(t, 1)-np.array([0., -.09, 0.]))[:2])))
        if max_error > np.deg2rad(1.):
            raise ValueError('fixed target orientation residual >1deg')
        if max_relative_speed > .02:
            raise ValueError('close relative XY speed >2cm/s')
        q = self.ready.copy(); q[14:21] = curve(3.)
        contact_rotation = self.robot.ee_fk('right', q)[:3, :3]
        plan['checks'].update(fixed_target_orientation_error_deg=float(np.degrees(max_error)),
                              max_close_relative_xy_speed_m_s=max_relative_speed)
        plan.update(target_rotation_base=self.target_rotation.tolist(),
                    actual_contact_rotation_base=contact_rotation.tolist())
        return plan


def worker(job):
    ix, iy, output = job
    builder = FixedRotationBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    point = [round(ix*.03, 8), round(iy*.03, 8), .97]
    records = []
    for rotation in rotations():
        builder.rng = np.random.default_rng(970)
        builder.target_rotation = Rotation.from_rotvec(np.deg2rad(rotation['rotvec_deg'])).as_matrix()@builder.rotation
        entry = dict(xy_m=point[:2], rotation_id=rotation['id'])
        try:
            # Necessary endpoint feasibility before the more costly complete path search.
            contact = builder.relaxed(np.array(point), builder.ready, attempts=2)
            builder.relaxed(target_position(point, 5., 1.5), contact, attempts=2)
        except ValueError as error:
            records.append(dict(entry, accepted=False, reason='endpoint: '+str(error)))
            continue
        failures = []
        for approach in (1.5, 1.8, 2., 1.2):
            try:
                plan = builder.fixed_plan(point, approach)
                plan['rotation_search'] = rotation
                name = f'x{ix:+04d}_y{iy:+04d}_{rotation["id"]}.json'
                path = Path(output)/'trajectories'/name
                path.write_text(json.dumps(plan, separators=(',', ':')))
                entry.update(accepted=True, file='trajectories/'+name,
                             sha256=digest(path), checks=plan['checks'], events=plan['events'])
                break
            except ValueError as error:
                failures.append(str(error))
        else:
            entry.update(accepted=False, reason=failures[-1], approach_failures=failures)
        records.append(entry)
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/right_grasp_rotation_search')
    parser.add_argument('--workers', type=int, default=12)
    parser.add_argument('--x-indices', type=int, nargs=2, default=[10, 24])
    parser.add_argument('--y-indices', type=int, nargs=2, default=[-6, 8])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'trajectories').mkdir(exist_ok=True)
    original = ROOT/'artifacts/right_grasp_grid_fast/manifest.json'
    old = json.loads(original.read_text())
    builder = FixedRotationBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    sources = [ROOT/'src/deux_task_bringup/config/installation.yaml', Path(builder.config['robot']['urdf_path']),
               ROOT/'calibration/conveyor_visible_surface.json', Path(__file__).resolve(),
               ROOT/'tools/build_fast_grasp_grid.py', ROOT/'tools/build_right_grasp_grid.py',
               ROOT/'tools/upper_collision_batch.py', ROOT/'src/deux_task_core/deux_task_core/model.py',
               ROOT/'src/deux_task_core/deux_task_core/conveyor_screen.py', original]
    manifest = dict(schema='deux.offline_fixed_rotation_sweep.v1', frame='base_link',
                    rotation_convention='R_target = Exp(rotvec_base) @ R_reference; geodesic norm <=30deg',
                    reference_rotation=builder.rotation.tolist(), rotations=rotations(),
                    ready_joints_28_rad=builder.ready.tolist(), contact_z_m=.97,
                    grid_step_m=.03, x_indices=args.x_indices, y_indices=args.y_indices,
                    contact_s=3., close_start_s=2.5, lift_complete_s=4.5, stopped_s=5.,
                    velocity_m_s=[0., -.09, 0.], complete=False,
                    inputs={str(p.relative_to(ROOT)):digest(p) for p in sources},
                    limitations=old['limitations']+[
                        'Finite rotation and XY search, not global reachability or optimality proof.',
                        'Fixed rotations tracked within 1deg; original 30deg cone checked independently.',
                        'Static left arm is installation home, not fresh hardware feedback.',
                        'Separate offline artifact, not compatible with live GridLibrary dispatch schema.'],
                    records=[])
    jobs = [(x, y, str(args.output)) for x in range(args.x_indices[0], args.x_indices[1]+1)
            for y in range(args.y_indices[0], args.y_indices[1]+1)]
    def checkpoint():
        manifest['records'].sort(key=lambda r:(r['xy_m'], r['rotation_id']))
        (args.output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(worker, job) for job in jobs]
        for count, future in enumerate(as_completed(futures), 1):
            manifest['records'].extend(future.result())
            if count%5 == 0 or count == len(jobs):
                checkpoint()
                print(json.dumps(dict(cells_done=count, cells_total=len(jobs),
                                      accepted=sum(r['accepted'] for r in manifest['records']))), flush=True)
    manifest['complete'] = True
    checkpoint()


if __name__ == '__main__':
    main()
