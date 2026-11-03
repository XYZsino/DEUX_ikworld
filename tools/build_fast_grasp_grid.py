"""Offline 9 cm/s -Y library, contact <=3 s, full orientation cone <=30 deg."""
import argparse
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from scipy.optimize import least_squares
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from build_right_grasp_grid import Builder, ROOT, digest
from upper_collision_batch import checked_clearances

V = np.array([0., -.09, 0.])
CONTACT = 3.
CLOSE = .5
LIFT = 1.5
BRAKE = .5


def smooth(u):
    return 10*u**3-15*u**4+6*u**5


def target_position(point, t, approach):
    p = np.array(point, float)
    if t <= CONTACT:
        p += V*(t-CONTACT)
        p[2] += .08*(1-smooth(np.clip((t-approach)/(CONTACT-approach), 0, 1)))
    elif t <= CONTACT+LIFT:
        p += V*(t-CONTACT)
        p[2] += .10*smooth((t-CONTACT)/LIFT)
    else:
        u = np.clip((t-CONTACT-LIFT)/BRAKE, 0, 1)
        # Integral of 1-smooth(u), continuous zero terminal velocity/acceleration.
        integral = u-2.5*u**4+3*u**5-u**6
        p += V*(LIFT+BRAKE*integral)
        p[2] += .10
    return p


class FastBuilder(Builder):
    def __init__(self, installation):
        super().__init__(installation)
        if not np.isclose(self.config.get('hand_close_duration_s', .5), CLOSE):
            raise ValueError('recompute closure timing for changed installation hand_close_duration_s')

    def relaxed(self, xyz, seed, attempts=1):
        reference = self.rotation
        rng = np.random.default_rng(970)
        for attempt in range(attempts):
            initial = seed[14:21] if attempt == 0 else rng.uniform(self.lo, self.hi)
            cache = {}
            def evaluate(arm):
                if 'arm' not in cache or not np.array_equal(cache['arm'], arm):
                    t, j = self.fk_jac(arm)
                    phi = Rotation.from_matrix(t[:3, :3]@reference.T).as_rotvec()
                    angle = np.linalg.norm(phi)
                    x, y, z = phi; k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
                    coefficient = 1/12 if angle < 1e-5 else (1-.5*angle/np.tan(.5*angle))/angle**2
                    inv = np.eye(3)-.5*k+coefficient*k@k
                    # Soft reference preference inside a hard independently checked cone.
                    excess = max(0., angle-np.deg2rad(29.5))
                    radial = phi/max(angle, 1e-12)
                    scale = .001 + .3*excess/max(angle, 1e-12)
                    derivative = scale*np.eye(3)
                    if excess: derivative += .3*np.deg2rad(29.5)/angle*np.outer(radial, radial)
                    cache.update(arm=arm.copy(), residual=np.r_[t[:3, 3]-xyz, scale*phi, .01*(arm-seed[14:21])],
                                 jac=np.vstack([j[:3], derivative@inv@j[3:], .01*np.eye(7)]), pose=t)
                return cache
            result = least_squares(lambda a: evaluate(a)['residual'], np.clip(initial, self.lo+1e-9, self.hi-1e-9),
                                   jac=lambda a: evaluate(a)['jac'], bounds=(self.lo, self.hi), max_nfev=60,
                                   ftol=1e-8, xtol=1e-8, gtol=1e-8)
            t = evaluate(result.x)['pose']
            if np.linalg.norm(t[:3, 3]-xyz) <= .0015 and Rotation.from_matrix(t[:3, :3]@reference.T).magnitude() <= np.deg2rad(30):
                q = seed.copy(); q[14:21] = result.x
                return q
        raise ValueError('position / 30deg orientation cone IK')

    def plan_fast(self, point, approach=1.5):
        # Closure overlaps descent and must finish at contact, without a bottom dwell.
        contact_q = self.relaxed(np.array(point), self.ready, attempts=2)
        self.relaxed(target_position(point, CONTACT-CLOSE, approach), contact_q)
        total = CONTACT+LIFT+BRAKE
        times = np.linspace(approach, total, round((total-approach)/.05)+1)
        q = self.relaxed(target_position(point, approach, approach), contact_q, attempts=2)
        path = [q.copy()]
        for t in times[1:]:
            q = self.relaxed(target_position(point, t, approach), q)
            if np.max(abs(q[14:21]-path[-1][14:21])) > .18:
                raise ValueError('IK branch discontinuity')
            path.append(q.copy())
        path = np.array(path)
        velocities = np.gradient(path, times, axis=0, edge_order=2)
        velocities[-1] = 0.
        # One continuous Hermite approach from the exact ready state to the moving descent.
        times = np.r_[0., times]; path = np.vstack([self.ready, path]); velocities = np.vstack([np.zeros(28), velocities])
        curve = CubicHermiteSpline(times, path, velocities)
        ratios = {}
        for order, limits in [(0, None), (1, self.robot.velocity), (2, self.robot.acceleration)]:
            peak_ratio = 0.
            for j in range(14, 21):
                scalar = CubicHermiteSpline(times, path[:, j], velocities[:, j]).derivative(order)
                roots = scalar.derivative().roots(extrapolate=False); roots = roots[np.isfinite(roots)]
                vals = scalar(np.r_[times, np.nextafter(times[1:], times[:-1]), roots])
                if order == 0:
                    if vals.min() < self.lo[j-14] or vals.max() > self.hi[j-14]: raise ValueError('continuous joint position limit')
                else: peak_ratio = max(peak_ratio, float(np.max(abs(vals))/limits[j]))
            if order:
                ratios[str(order)] = peak_ratio
                if peak_ratio > .9: raise ValueError('continuous '+('velocity' if order == 1 else 'acceleration')+' >90% limit')
        sample_times = np.linspace(0, total, round(total/.01)+1)
        states = curve(sample_times)
        margin = float(np.min(checked_clearances(self.robot, states, self.config['obstacles'], self.config['collision_margin'])))
        position_error = angle_error = 0.
        for t, state in zip(sample_times, states):
            if t < approach-1e-9: continue
            actual = self.robot.ee_fk('right', state)
            position_error = max(position_error, float(np.linalg.norm(actual[:3, 3]-target_position(point, t, approach))))
            angle_error = max(angle_error, float(Rotation.from_matrix(actual[:3, :3]@self.rotation.T).magnitude()))
        if position_error > .003: raise ValueError('interpolated position residual >3mm')
        if angle_error > np.deg2rad(30): raise ValueError('interpolated orientation cone >30deg')
        return dict(frame='base_link', contact_xyz_m=list(point), velocity_m_s=V.tolist(),
                    joint_names=[f'arm_r_joint{i}' for i in range(1, 8)],
                    times_s=times.tolist(), positions_rad=path[:, 14:21].tolist(), velocities_rad_s=velocities[:, 14:21].tolist(),
                    interpolation='cubic_hermite_positions_and_velocities',
                    events=dict(approach_complete_s=approach, close_start_s=CONTACT-CLOSE,
                                contact_s=CONTACT, close_complete_s=CONTACT, lift_start_s=CONTACT,
                                lift_complete_s=CONTACT+LIFT, stopped_s=total),
                    hand_schedule=dict(ready_fraction=float(self.config.get('right_hand_ready_fraction', 0.)),
                                       close_fraction=1., close_start_s=CONTACT-CLOSE, close_duration_s=CLOSE,
                                       close_complete_s=CONTACT, parallel_with='Descend',
                                       late_command_policy='abort; never shift closure after contact',
                                       closure_duration_source='installation.yaml hand_close_duration_s nominal; physical completion unverified',
                                       interface='existing checked RightTrialHand adapter', repeat_period_s=.02),
                    checks=dict(position_error_m=position_error, orientation_error_deg=float(np.degrees(angle_error)),
                                velocity_limit_ratio=ratios['1'], acceleration_limit_ratio=ratios['2'],
                                min_measured_surface_margin_m=margin),
                    dispatch=dict(grasp_point_at_start_m=(np.array(point)-V*CONTACT).tolist(),
                                  note='contact grid represents the grasp point at t=3s, not the position at dispatch'))


def worker(item):
    ix, iy, output = item; x, y = round(ix*.03, 8), round(iy*.03, 8)
    builder = FastBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    reason = ''
    for approach in [1.5, 1.8, 2., 1.2]:
        try:
            p = builder.plan_fast([x, y, .97], approach)
            name = f'x{ix:+04d}_y{iy:+04d}.json'
            path = Path(output)/name; path.write_text(json.dumps(p, separators=(',', ':')))
            return True, dict(xy_m=[x, y], file=name, sha256=digest(path), checks=p['checks'], events=p['events'])
        except ValueError as error:
            reason = str(error)
            if reason == 'position / 30deg orientation cone IK': break
    return False, dict(xy_m=[x, y], reason=reason)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/right_grasp_grid_fast')
    parser.add_argument('--workers', type=int, default=4)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    b = FastBuilder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    lo = np.ceil(np.array(b.config['workspace']['min'])[:2]/.03).astype(int)
    hi = np.floor(np.array(b.config['workspace']['max'])[:2]/.03).astype(int)
    inputs = [ROOT/'src/deux_task_bringup/config/installation.yaml', Path(b.config['robot']['urdf_path']), ROOT/'calibration/conveyor_visible_surface.json', Path(__file__).resolve()]
    m = dict(schema='deux.fast_grasp_grid.v2', frame='base_link', grid_step_m=.03, grasp_z_m=.97,
             velocity_m_s=V.tolist(), contact_s=CONTACT, orientation_cone_deg=30.,
             close_start_s=CONTACT-CLOSE, close_complete_s=CONTACT, lift_start_s=CONTACT,
             reference_rotation=b.rotation.tolist(), ready_joints_28_rad=b.ready.tolist(),
             search_xy_bounds_m=[(lo*.03).tolist(), (hi*.03).tolist()],
             inputs={str(p.relative_to(ROOT)):digest(p) for p in inputs},
             verification='offline model evidence only; no hardware commands',
             limitations=['Existing grasp spheres and observed conveyor envelopes only; full body/self/held-object collision unverified.',
                          'Approach is a joint-space cubic to moving descent; no Cartesian straight-line guarantee during approach.',
                          'Position/orientation and collision sampled every 10ms; joint polynomial extrema checked analytically.',
                          'Cone applies from descent start through lifted stop; ready orientation may lie outside cone.',
                          'Fixed -Y speed and fixed object orientation; velocity changes require another validated library.',
                          'Hand closure is a command schedule, not physical contact verification; terminal stop is included.'],
             accepted=[], rejected=[])
    jobs = [(ix, iy, str(args.output)) for ix in range(lo[0], hi[0]+1) for iy in range(lo[1], hi[1]+1)]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for index, (ok, entry) in enumerate(pool.map(worker, jobs, chunksize=1), 1):
            m['accepted' if ok else 'rejected'].append(entry)
            if index % 67 == 0 or index == len(jobs):
                (args.output/'manifest.json').write_text(json.dumps(m, indent=2))
                print(f'{index}/{len(jobs)} accepted={len(m["accepted"])}', flush=True)


if __name__ == '__main__': main()
