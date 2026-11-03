"""Pure stored-grid selection and replay contract. No ROS or hardware calls."""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from scipy.spatial.transform import Rotation
from build_right_grasp_grid import ROOT
from deux_task_core.model import RobotModel

XY_TOLERANCE_M = .03


def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class GridLibrary:
    def __init__(self, config, root=ROOT, recommended=True, stationary_left_file=None,
                 directory=None, yaw_secondary=False, load_rotation_paths=True):
        self.root = Path(root)
        directory = Path(directory) if directory else self.root/'artifacts/right_grasp_grid_fast'
        if not directory.is_absolute(): directory = self.root/directory
        self.directory = directory; self.yaw_secondary = yaw_secondary
        self.manifest = json.loads((directory/'manifest.json').read_text())
        m = self.manifest
        if m['schema'] != 'deux.fast_grasp_grid.v2': raise ValueError('parallel-close v2 library required')
        for path, digest in m['inputs'].items():
            if sha(self.root/path) != digest: raise ValueError('library input changed: '+path)
        self.robot = RobotModel(config['robot']); self.config = config
        self.ready = np.array(m['ready_joints_28_rad']); self.velocity = np.array(m['velocity_m_s'])
        self.grasp_z = float(m['grasp_z_m'])
        session = None
        if stationary_left_file:
            session = json.loads(Path(stationary_left_file).read_text())
            if session['manifest_sha256'] != sha(directory/'manifest.json'):
                raise ValueError('stationary-left audit belongs to another library')
            left = np.asarray(session['left_arm_rad'], float)
            if left.shape != (7,) or not np.isfinite(left).all(): raise ValueError('invalid stationary left snapshot')
            self.ready[:7] = left
        self.offset = np.array(config['grasp_object_from_right_ee'])
        self.entries = {}; self.curves = {}; self.contact_rotations = {}; self.rotation_paths = {}
        allowed = None
        if recommended:
            allowed = {p['file'] for p in json.loads((directory/'recommended_cells.json').read_text())['cells']}
        for item in m['accepted']:
            path = directory/item['file']
            if sha(path) != item['sha256']: raise ValueError('trajectory hash mismatch: '+str(path))
            p = json.loads(path.read_text()); e = p['events']; h = p['hand_schedule']
            if (e['contact_s'] != 3. or e['close_start_s'] != 2.5 or e['close_complete_s'] != 3.
                    or e['lift_start_s'] != 3. or h['close_start_s']+h['close_duration_s'] != 3.):
                raise ValueError('invalid parallel closure event timing')
            if allowed is not None and item['file'] not in allowed: continue
            key = path.stem
            if session is not None and key not in session['validated_cells']:
                if session.get('complete_library_audit') and key in session.get('rejected_cells', {}): continue
                raise ValueError('cell was not collision-screened for this stationary left pose')
            curve = CubicHermiteSpline(p['times_s'], p['positions_rad'], p['velocities_rad_s'])
            if np.max(abs(curve(0)-self.ready[14:21])) > 1e-10: raise ValueError('ready mismatch')
            q = self.ready.copy(); q[14:21] = curve(3.)
            self.entries[key] = p; self.curves[key] = curve
            self.contact_rotations[key] = self.robot.ee_fk('right', q)[:3, :3]
            if load_rotation_paths:
                rotations = []
                for t in np.arange(p['events']['approach_complete_s'], p['events']['stopped_s']+.001, .01):
                    q = self.ready.copy(); q[14:21] = curve(t)
                    rotations.append(self.robot.ee_fk('right', q)[:3, :3])
                self.rotation_paths[key] = np.array(rotations)
        if not self.entries: raise ValueError('empty grid library')
        self.cell_options = {}
        for key, plan in self.entries.items():
            self.cell_options.setdefault(tuple(plan['contact_xyz_m']), []).append(key)

    def grasp_now(self, object_pose, velocity, stamp, now):
        if not 0 <= now-stamp <= .5: raise ValueError('prediction acquisition age >0.5s / future stamp')
        obj = np.array(object_pose, float); v = np.array(velocity, float)
        if obj.shape != (4, 4) or v.shape != (4,) or not np.isfinite(obj).all() or not np.isfinite(v).all():
            raise ValueError('invalid observation')
        # The estimator reports MODEL CENTER height. The user explicitly fixed
        # the physical grasp plane in base_link; do not equate those two origins.
        v[2] = 0.
        # Explicit operator instruction: select by observed XY only. Belt speed
        # and fixed object orientation are known inputs, not ICP admission gates.
        v = np.r_[self.velocity, 0.]
        # Object orientation is assumed fixed as requested; only measured XY
        # and the operator-specified belt speed affect cell/arrival selection.
        grasp = obj@self.offset; grasp[:3, 3] += v[:3]*(now-stamp)
        grasp[2, 3] = self.grasp_z
        return grasp, v

    def select(self, object_pose, velocity, stamp, now, lead=.8):
        grasp, v = self.grasp_now(object_pose, velocity, stamp, now)
        if self.yaw_secondary:
            return self.select_with_yaw(grasp, object_pose, stamp, now, lead)
        choices = []
        for key, p in self.entries.items():
            cell = np.array(p['contact_xyz_m']); eta = (cell[1]-grasp[1, 3])/v[1]
            if not 3.+lead <= eta <= 10.: continue
            future = grasp[:3, 3]+v[:3]*eta
            lateral = abs(float(future[0]-cell[0])); height = abs(float(future[2]-cell[2]))
            angle = float(np.deg2rad(self.entries[key]['checks']['orientation_error_deg']))
            if lateral > XY_TOLERANCE_M+1e-9: continue
            choices.append((lateral, eta, key, height, angle))
        if not choices: raise ValueError('no cell with sufficient lead time / XY alignment')
        lateral, eta, key, height, angle = min(choices)
        return dict(cell_id=key, contact_xyz_m=self.entries[key]['contact_xyz_m'],
                    start_at_s=now+eta-3., contact_at_s=now+eta,
                    start_in_s=eta-3., contact_in_s=eta, lateral_error_m=lateral,
                    height_error_m=height, rotation_error_deg=float(np.degrees(angle)),
                    acquired_at_s=stamp, grasp_now_m=grasp[:3, 3].tolist(), velocity_m_s=v[:3].tolist())

    def select_with_yaw(self, grasp, object_pose, stamp, now, lead):
        # Preserve spatial and arrival ordering EXACTLY. Rotation only ranks
        # alternatives at the chosen contact cell, never moves that cell/time.
        choices = []
        for cell in self.cell_options:
            eta = (cell[1]-grasp[1, 3])/self.velocity[1]
            lateral = abs(float(grasp[0, 3]-cell[0]))
            if 3.+lead <= eta <= 10. and lateral <= XY_TOLERANCE_M+1e-9:
                choices.append((lateral, eta, cell))
        if not choices: raise ValueError('no cell with sufficient lead time / XY alignment')
        lateral, eta, cell = min(choices)
        pose = np.asarray(object_pose)
        yaw_available = np.hypot(pose[0, 0], pose[1, 0]) > 1e-6
        yaw = float(np.arctan2(pose[1, 0], pose[0, 0])) if yaw_available else 0.
        desired = Rotation.from_euler('z', yaw).as_matrix()@self.offset[:3, :3]
        keys = self.cell_options[cell]
        rotations = np.asarray([self.contact_rotations[key] for key in keys])
        cosines = (np.einsum('nij,ij->n', rotations, desired)-1.)/2.
        angles = np.arccos(np.clip(cosines, -1., 1.))
        options = [(float(angle), self.entries[key]['checks']['acceleration_limit_ratio'], key)
                   for key, angle in zip(keys, angles)]
        error, acceleration, key = min(options)
        p = self.entries[key]
        return dict(cell_id=key, contact_xyz_m=list(cell), start_at_s=now+eta-3., contact_at_s=now+eta,
                    start_in_s=eta-3., contact_in_s=eta, lateral_error_m=lateral,
                    height_error_m=abs(float(grasp[2, 3]-cell[2])),
                    rotation_error_deg=p['checks']['orientation_error_deg'],
                    observed_yaw_deg=float(np.degrees(yaw)), yaw_available=bool(yaw_available),
                    yaw_target_rotation_error_deg=float(np.degrees(error)),
                    selected_rotation=p.get('rotation_search'), rotation_candidates_at_cell=len(keys),
                    acceleration_limit_ratio=acceleration,
                    selection_priority='minimum X error, earliest eligible ETA, then nearest yaw-only target rotation, then acceleration',
                    rotation_is_soft_preference=True, acquired_at_s=stamp,
                    grasp_now_m=grasp[:3, 3].tolist(), velocity_m_s=self.velocity.tolist())

    def live_error(self, selected, object_pose, velocity, stamp, now):
        grasp, v = self.grasp_now(object_pose, velocity, stamp, now)
        expected = np.array(selected['contact_xyz_m'])+self.velocity*(now-selected['contact_at_s'])
        error = grasp[:3, 3]-expected
        if np.linalg.norm(error[:2]) > XY_TOLERANCE_M+1e-9:
            raise ValueError('live prediction departed selected stored path >30mm')
        key = selected['cell_id']
        return error
