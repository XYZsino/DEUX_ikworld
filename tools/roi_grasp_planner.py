"""ROS-free ROI alignment and fixed-speed timed grasp planning.

Every dispatch is a complete, stopped path. Never truncate a moving grid path.
"""
import copy
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import BPoly, CubicHermiteSpline, CubicSpline
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
from build_fast_grasp_grid import FastBuilder, V, target_position
from upper_collision_batch import checked_clearances

DEFAULT_POLICY = Path(__file__).resolve().parents[1]/'calibration/roi_grasp_policy.json'


def load_policy(path=DEFAULT_POLICY):
    p = json.loads(Path(path).read_text())
    for name in ('x_tolerance_m', 'x_deadband_m', 'dispatch_lead_s', 'alignment_min_s', 'upper_z_m'):
        if not np.isfinite(p[name]) or p[name] <= 0: raise ValueError('invalid '+name)
    if p['x_deadband_m'] > p['x_tolerance_m']: raise ValueError('deadband exceeds tolerance')
    bounds = np.asarray(p['grasp_entry_xy_m'], float)
    if bounds.shape != (2, 2) or not np.isfinite(bounds).all() or np.any(bounds[0] >= bounds[1]):
        raise ValueError('invalid grasp entry bounds')
    if not p['descent_durations_s'] or any(not np.isfinite(t) or t < .5 for t in p['descent_durations_s']):
        raise ValueError('descent must include full 0.5s closure')
    if not np.isfinite(p.get('planning_budget_s', .5)) or p.get('planning_budget_s', .5) < 0:
        raise ValueError('invalid planning budget')
    if not np.isfinite(p['standby_y_m']): raise ValueError('invalid standby Y')
    return p


def encounter(grasp, actual_x, cells, duration, now, policy):
    """Select by current entry region, measured X alignment, and future contact."""
    xy = np.asarray(grasp, float)[:2]
    low, high = np.asarray(policy['grasp_entry_xy_m'])
    if np.any(xy < low) or np.any(xy > high): raise ValueError('outside grasp entry region')
    if abs(xy[0]-actual_x) > policy['x_tolerance_m']: raise ValueError('X alignment incomplete')
    choices = []
    for key, cell in cells.items():
        xyz = np.asarray(cell['contact_xyz_m'])
        eta = (xy[1]-xyz[1])/.09
        error = abs(xy[0]-xyz[0])
        if error <= policy['x_tolerance_m'] and eta >= duration+policy['dispatch_lead_s']:
            choices.append((error, eta, key))
    if not choices: raise ValueError('no reachable cell with descent lead time')
    # Equal-X cells: retain the largest arrival margin for live IK/validation.
    _, eta, key = min(choices, key=lambda choice: (choice[0], -choice[1], choice[2]))
    return dict(cell_id=key, contact_xyz_m=cells[key]['contact_xyz_m'],
                start_at_s=now+eta-duration, contact_at_s=now+eta,
                start_in_s=eta-duration, contact_in_s=eta, descent_duration_s=duration)


class RoiPlanner(FastBuilder):
    def __init__(self, config, policy):
        # Runtime already resolves model paths; no .setup or machine-specific inputs.
        from deux_task_core.model import RobotModel
        self.config = copy.deepcopy(config); self.policy = policy
        self.robot = RobotModel(config['robot'])
        self.ready = np.asarray(config['home_joints'], float)
        self.rotation = np.asarray(config['grasp_object_from_right_ee'])[:3, :3]
        self.lo = self.robot.limits[14:21, 0]+.003
        self.hi = self.robot.limits[14:21, 1]-.003

    def solve(self, xyz, seed):
        # Redundant 7-DOF IK needs a posture term to avoid knot-to-knot nullspace jumps.
        anchor = seed[14:21]
        def residual(arm):
            pose, _ = self.fk_jac(arm)
            phi = Rotation.from_matrix(pose[:3, :3]@self.rotation.T).as_rotvec()
            return np.r_[pose[:3, 3]-xyz, .15*phi, .003*(arm-anchor)]
        def jacobian(arm):
            pose, jac = self.fk_jac(arm)
            phi = Rotation.from_matrix(pose[:3, :3]@self.rotation.T).as_rotvec()
            angle = np.linalg.norm(phi)
            x, y, z = phi; k = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
            c = 1/12 if angle < 1e-5 else (1-.5*angle/np.tan(.5*angle))/angle**2
            return np.vstack([jac[:3], .15*(np.eye(3)-.5*k+c*k@k)@jac[3:], .003*np.eye(7)])
        fit = least_squares(residual, np.clip(anchor, self.lo+1e-9, self.hi-1e-9),
                            jac=jacobian, bounds=(self.lo, self.hi), max_nfev=60,
                            ftol=1e-10, xtol=1e-10, gtol=1e-10)
        error = residual(fit.x)
        if np.linalg.norm(error[:3]) > .001 or np.linalg.norm(error[3:6])/.15 > np.deg2rad(1):
            raise ValueError('locked orientation IK failed')
        q = seed.copy(); q[14:21] = fit.x
        return q

    def certify(self, times, path, velocities, desired=None, rotation=None):
        curve = CubicHermiteSpline(times, path, velocities)
        ratios = {}
        for order in (0, 1, 2):
            peak = 0.
            for j in range(14, 21):
                scalar = CubicHermiteSpline(times, path[:, j], velocities[:, j]).derivative(order)
                roots = scalar.derivative().roots(extrapolate=False); roots = roots[np.isfinite(roots)]
                values = scalar(np.r_[times, np.nextafter(times[1:], times[:-1]), roots])
                if order == 0:
                    if values.min() < self.lo[j-14] or values.max() > self.hi[j-14]: raise ValueError('joint position limit')
                else:
                    limits = self.robot.velocity if order == 1 else self.robot.acceleration
                    peak = max(peak, float(np.max(abs(values))/limits[j]))
            if order and peak > .9: raise ValueError(f'joint derivative {order} limit ratio {peak:.3f}')
            ratios[str(order)] = peak
        tt = np.linspace(0, times[-1], int(np.ceil(times[-1]/.01))+1)
        states = curve(tt)
        clearance = float(np.min(checked_clearances(self.robot, states, self.config['obstacles'], self.config['collision_margin'])))
        residual = angle = 0.
        for t, q in zip(tt, states):
            pose = self.robot.ee_fk('right', q)
            if desired is not None: residual = max(residual, float(np.linalg.norm(pose[:3, 3]-desired(t))))
            if rotation is not None: angle = max(angle, float(Rotation.from_matrix(pose[:3, :3]@rotation.T).magnitude()))
        if desired is not None and residual > .003: raise ValueError('Cartesian residual >3mm')
        if rotation is not None and angle > np.deg2rad(2): raise ValueError('locked rotation residual >2deg')
        return dict(joint_names=[f'arm_r_joint{i}' for i in range(1, 8)], times_s=times.tolist(),
                    positions_rad=path[:, 14:21].tolist(), velocities_rad_s=velocities[:, 14:21].tolist(),
                    checks=dict(position_error_m=residual, rotation_error_rad=angle, limit_ratios=ratios,
                                min_measured_surface_margin_m=clearance),
                    events=dict(stopped_s=float(times[-1])))

    def align(self, start, x):
        target = np.array([x, self.policy['standby_y_m'], self.policy['upper_z_m']])
        end = self.relaxed(target, start, attempts=2)
        # Stopped cubic move. First acquisition also raises/moves to the waiting line.
        # Subsequent updates retain fixed upper Y/Z and change X only at endpoints.
        delta = abs(end[14:21]-start[14:21])
        duration = max(self.policy['alignment_min_s'],
                       float(np.max(1.5*delta/(.7*self.robot.velocity[14:21]))),
                       float(np.sqrt(np.max(6*delta/(.7*self.robot.acceleration[14:21])))))
        pose = self.robot.ee_fk('right', start)
        if np.linalg.norm(pose[1:3, 3]-target[1:3]) <= .003:
            # Once at the waiting line, track only X at fixed Y/Z and rotation.
            self.rotation = pose[:3, :3].copy()
            target[1:3] = pose[1:3, 3]
            u = np.linspace(0., 1., 21)
            def point(t):
                v = np.clip(t/duration, 0., 1.)
                return pose[:3, 3]+(target-pose[:3, 3])*(10*v**3-15*v**4+6*v**5)
            rows = [start]
            for v in u[1:]: rows.append(self.solve(point(v*duration), rows[-1]))
            path = np.array(rows)
            # Uniform time scaling preserves geometry; analytic extrema determine acceptance.
            for _ in range(8):
                times = u*duration
                velocities = CubicSpline(times, path, bc_type=((1, np.zeros(28)), (1, np.zeros(28))))(times, 1)
                try:
                    p = self.certify(times, path, velocities, point, self.rotation); break
                except ValueError as error:
                    if 'derivative' not in str(error): raise
                    duration *= 1.3
            else: raise ValueError('X alignment retiming exhausted')
        else:
            p = self.certify(np.array([0., duration]), np.array([start, end]), np.zeros((2, 28)))
        p['alignment_xyz_m'] = target.tolist()
        return p

    def grasp(self, start, cell, duration):
        pose = self.robot.ee_fk('right', start); point = np.asarray(cell, float)
        if pose[2, 3] <= point[2]: raise ValueError('start is not above grasp plane')
        # Lock actual aligned rotation throughout descent, contact, lift and brake.
        self.rotation = pose[:3, :3].copy()
        descent = BPoly.from_derivatives([0., duration],
                    [[pose[:3, 3], np.zeros(3), np.zeros(3)], [point, V, np.zeros(3)]])
        def desired(t):
            if t <= duration: return descent(t)
            return target_position(point, t-duration+3., 1.5)
        total = duration+2.
        times = np.unique(np.round(np.r_[np.arange(0., total, .05), duration-.5, duration, total], 9))
        path = [np.array(start, float)]
        for t in times[1:]:
            q = self.solve(desired(t), path[-1])
            if np.max(abs(q[14:21]-path[-1][14:21])) > .18: raise ValueError('IK branch discontinuity')
            path.append(q)
        path = np.array(path); velocities = CubicSpline(times, path, bc_type=((1, np.zeros(28)), (1, np.zeros(28))))(times, 1)
        p = self.certify(times, path, velocities, desired, self.rotation)
        p.update(contact_xyz_m=point.tolist(), velocity_m_s=V.tolist(),
                 hand_schedule=dict(close_start_s=duration-.5, close_duration_s=.5, close_fraction=1., repeat_period_s=.02))
        p['events'].update(contact_s=duration, close_start_s=duration-.5, close_complete_s=duration,
                           lift_start_s=duration, lift_complete_s=duration+1.5)
        return p
