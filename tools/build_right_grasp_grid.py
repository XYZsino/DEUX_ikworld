"""Offline 3 cm grasp library. No ROS imports, clients, publishers or motion.

Run with /usr/bin/python3 tools/build_right_grasp_grid.py.
Each JSON stores independent, stopped cubic-Hermite stages for the right arm.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from scipy.interpolate import CubicHermiteSpline

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/deux_task_core'))
sys.path.insert(0, str(ROOT / 'tools'))
from deux_task_core.model import RobotModel, ARM_NAMES
from upper_collision_batch import checked_clearances


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Builder:
    def __init__(self, installation):
        self.config = yaml.safe_load(installation.read_text())
        c = self.config
        c['robot']['urdf_path'] = str((installation.parent / c['robot']['urdf_path']).resolve())
        self.robot = RobotModel(c['robot'])
        self.ready = np.array(c['home_joints'], float)
        self.rotation = np.array(c['grasp_object_from_right_ee'])[:3, :3]
        self.lo = self.robot.limits[14:21, 0] + .003
        self.hi = self.robot.limits[14:21, 1] - .003
        self.rng = np.random.default_rng(970)
        self.seed = self.ready.copy()

    def fk_jac(self, arm):
        t = np.eye(4); axes = []; origins = []
        values = dict(zip(ARM_NAMES[7:], arm))
        for name, joint in self.robot.ee_chains['right']:
            t = t @ joint['origin']
            if name in values:
                axes.append(t[:3, :3] @ joint['axis']); origins.append(t[:3, 3].copy())
                m = np.eye(4)
                m[:3, :3] = Rotation.from_rotvec(joint['axis'] * values[name]).as_matrix()
                t = t @ m
            elif joint['kind'] != 'fixed':
                raise ValueError('unexpected unfixed joint: ' + name)
        axes = np.array(axes).T
        return t, np.vstack([np.cross(axes.T, t[:3, 3] - origins).T, axes])

    def solve(self, xyz, seed, retries=0):
        cache = {}
        def evaluate(arm):
            if 'arm' not in cache or not np.array_equal(cache['arm'], arm):
                t, j = self.fk_jac(arm)
                phi = Rotation.from_matrix(t[:3, :3] @ self.rotation.T).as_rotvec()
                a = np.linalg.norm(phi)
                x, y, z = phi
                k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
                coefficient = 1/12 if a < 1e-5 else (1 - .5*a/np.tan(.5*a))/a**2
                inv = np.eye(3) - .5*k + coefficient*k@k
                cache.update(arm=arm.copy(), residual=np.r_[t[:3, 3]-xyz, .15*phi],
                             jac=np.vstack([j[:3], .15*inv@j[3:]]))
            return cache
        seeds = [seed[14:21], self.ready[14:21]]
        seeds += [self.rng.uniform(self.lo, self.hi) for _ in range(retries)]
        for arm in seeds[:1+retries]:
            result = least_squares(lambda a: evaluate(a)['residual'], np.clip(arm, self.lo+1e-9, self.hi-1e-9),
                                   jac=lambda a: evaluate(a)['jac'], bounds=(self.lo, self.hi), max_nfev=65,
                                   ftol=1e-9, xtol=1e-9, gtol=1e-9)
            error = evaluate(result.x)['residual']
            if np.linalg.norm(error[:3]) <= .001 and np.linalg.norm(error[3:])/.15 <= np.deg2rad(1):
                q = seed.copy(); q[14:21] = result.x
                return q
        raise ValueError('fixed orientation IK failed (1mm / 1deg)')

    def stage(self, name, path, desired=None, minimum=1.):
        path = np.array(path); u = np.linspace(0, 1, len(path))
        # Zero velocity at every Cartesian knot: conservative, reproducible stops.
        # Cubic interpolation is explicitly exported, never re-solved at replay.
        v = np.zeros_like(path)
        curve = CubicHermiteSpline(u, path, v)
        duration = minimum
        for order, limits in [(1, self.robot.velocity), (2, self.robot.acceleration)]:
            for j in range(14, 21):
                f = curve.derivative(order)
                roots = CubicHermiteSpline(u, path[:, j], v[:, j]).derivative(order+1).roots(extrapolate=False)
                roots = roots[np.isfinite(roots)]
                peak = np.max(abs(f(np.r_[u, np.nextafter(u[1:], u[:-1]), roots])[:, j]))
                duration = max(duration, (peak/(.5*limits[j]))**(1/order))
        times = u*duration
        curve = CubicHermiteSpline(times, path, v)
        samples = curve(np.linspace(0, duration, int(np.ceil(duration/.01))+1))
        clearance = float(np.min(checked_clearances(self.robot, samples, self.config['obstacles'], self.config['collision_margin'])))
        max_pos = max_angle = 0.
        if desired is not None:
            for i, q in enumerate(samples):
                actual = self.robot.ee_fk('right', q)
                wanted = desired(i/(len(samples)-1))
                max_pos = max(max_pos, float(np.linalg.norm(actual[:3, 3]-wanted)))
                max_angle = max(max_angle, float(Rotation.from_matrix(actual[:3, :3]@self.rotation.T).magnitude()))
            if max_pos > .002 or max_angle > np.deg2rad(2):
                raise ValueError(name + ': interpolated Cartesian residual')
        return dict(name=name, duration_s=duration, times_s=times.tolist(),
                    positions_rad=path[:, 14:21].tolist(), velocities_rad_s=v[:, 14:21].tolist(),
                    min_measured_surface_margin_m=clearance,
                    max_position_error_m=max_pos if desired is not None else None,
                    max_rotation_error_rad=max_angle if desired is not None else None)

    def plan(self, x, y):
        z = .97; upper = z + self.config['approach_height']; lift = z + self.config['lift_height']
        q = self.solve(np.array([x, y, upper]), self.seed, retries=3)
        upper_q = q.copy()
        # Deterministic identical ready pose; monotone cubic joint approach.
        approach = self.stage('Approach', [self.ready, upper_q], minimum=3.)
        stages = [approach]
        for name, start, end in [('Descend', upper, z), ('Lift', z, lift)]:
            path = [q.copy()]
            zs = np.linspace(start, end, int(np.ceil(abs(end-start)/.005))+1)
            for height in zs[1:]:
                q = self.solve(np.array([x, y, height]), q)
                if np.max(abs(q[14:21]-path[-1][14:21])) > .15:
                    raise ValueError(name + ': discontinuous IK branch')
                path.append(q.copy())
            # Each knot uses smoothstep; desired Z follows the same parameter.
            def desired(u):
                w = min(u*(len(zs)-1), len(zs)-1-1e-12); k = int(w); s = w-k
                return np.array([x, y, zs[k]+(3*s*s-2*s*s*s)*(zs[k+1]-zs[k])])
            stage = self.stage(name, path, desired, minimum=abs(end-start)/.04)
            if name == 'Lift':
                hold = dict(name='Close', duration_s=float(self.config.get('hand_close_duration_s', .5)),
                            times_s=[0., float(self.config.get('hand_close_duration_s', .5))],
                            positions_rad=[path[0][14:21].tolist()]*2, velocities_rad_s=[[0.]*7]*2,
                            hand_command=dict(interface='RightTrialHand.send', fraction=1., repeat_period_s=.02))
                stages.append(hold)
            stages.append(stage)
        self.seed = upper_q
        return dict(frame='base_link', grasp_xyz_m=[x, y, z], joint_names=list(ARM_NAMES[7:]),
                    interpolation='cubic_hermite_positions_and_velocities', stages=stages,
                    initial_hand_command=dict(interface='RightTrialHand.open_ready', fraction=float(self.config.get('right_hand_ready_fraction', 0.))),
                    duration_s=sum(s['duration_s'] for s in stages))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/right_grasp_grid')
    parser.add_argument('--limit', type=int, help='debug only: limit candidate count')
    args = parser.parse_args()
    installation = ROOT/'src/deux_task_bringup/config/installation.yaml'
    b = Builder(installation); out = args.output; out.mkdir(parents=True, exist_ok=True)
    c = b.config
    # Full configured scene XY bounds, snapped to the base-origin 3 cm lattice.
    lo = np.ceil(np.array(c['workspace']['min'])[:2]/.03).astype(int)
    hi = np.floor(np.array(c['workspace']['max'])[:2]/.03).astype(int)
    manifest = dict(schema='deux.right_grasp_grid.v1', frame='base_link', grid_step_m=.03, grasp_z_m=.97,
                    installation_sha256=digest(installation), urdf_sha256=digest(c['robot']['urdf_path']),
                    measured_surface_sha256=digest(ROOT/'calibration/conveyor_visible_surface.json'),
                    backend='explicit offline scipy least_squares with analytic URDF Jacobian; installation backend unchanged',
                    fixed_grasp_rotation=b.rotation.tolist(), ready_joints_28_rad=b.ready.tolist(),
                    approach_height_m=c['approach_height'], lift_height_m=c['lift_height'],
                    search_xy_bounds_m=[(lo*.03).tolist(), (hi*.03).tolist()],
                    verification='offline URDF and measured visible-surface screening; no hardware verification',
                    limitations=['Configured collision spheres cover grasp frames only; full body/self collision is not certified.',
                                 'Measured conveyor surfaces omit hidden geometry; held object volume is not specified.',
                                 'Rejected points mean this deterministic solver/path failed, not a proof of unreachability.',
                                 'Cubic Hermite interpolation and stopped stage boundaries must be preserved during replay.',
                                 'Library starts at ready and ends lifted; no inter-cell transition is included.'],
                    accepted=[], rejected=[])
    total = 0
    for ix in range(lo[0], hi[0]+1):
        for iy in range(lo[1], hi[1]+1):
            x, y = round(ix*.03, 8), round(iy*.03, 8)
            if args.limit and total >= args.limit: break
            total += 1
            key = f'x{ix:+04d}_y{iy:+04d}'
            try:
                plan = b.plan(x, y)
                filename = key+'.json'; (out/filename).write_text(json.dumps(plan, separators=(',', ':')))
                manifest['accepted'].append(dict(id=key, xy_m=[x, y], file=filename, sha256=digest(out/filename), duration_s=plan['duration_s']))
            except ValueError as e:
                manifest['rejected'].append(dict(xy_m=[x, y], reason=str(e)))
        (out/'manifest.json').write_text(json.dumps(manifest, indent=2))
        print(f'x={ix*.03:.2f}: accepted={len(manifest["accepted"])} rejected={len(manifest["rejected"])}', flush=True)
        if args.limit and total >= args.limit: break
    print(out/'manifest.json')


if __name__ == '__main__':
    main()
