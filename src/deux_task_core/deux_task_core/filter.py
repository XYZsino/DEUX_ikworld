"""Constant-velocity XY/yaw filter: one update per original acquisition identity."""

import numpy as np


class PoseFilter:
    def __init__(self, process_noise=0.2, measurement_noise=0.01, max_gap=0.6):
        self.process_noise, self.measurement_noise, self.max_gap = (
            process_noise,
            measurement_noise,
            max_gap,
        )
        self.tracks = {}

    def update(self, identity, stamp, sequence, xyz_yaw):
        z = np.asarray(xyz_yaw, dtype=float)[[0, 1, 3]]
        old = self.tracks.get(identity)
        if old is not None and stamp <= old[0]:
            return old[2][:3].copy(), False
        if old is None or stamp - old[0] > self.max_gap:
            x = np.r_[z, np.zeros(3)]
            cov = np.eye(6) * self.measurement_noise
        else:
            dt = stamp - old[0]
            x, cov = old[2].copy(), old[3].copy()
            z[2] = x[2] + np.arctan2(np.sin(z[2] - x[2]), np.cos(z[2] - x[2]))
            f = np.eye(6)
            f[:3, 3:] = np.eye(3) * dt
            g = np.vstack([np.eye(3) * dt * dt / 2, np.eye(3) * dt])
            x = f @ x
            cov = f @ cov @ f.T + g @ g.T * self.process_noise**2
            h = np.eye(6)[:3]
            r = np.eye(3) * self.measurement_noise
            k = np.linalg.solve((h @ cov @ h.T + r).T, (cov @ h.T).T).T
            x += k @ (z - h @ x)
            a = np.eye(6) - k @ h
            cov = a @ cov @ a.T + k @ r @ k.T
        self.tracks[identity] = (stamp, sequence, x, cov)
        self.tracks = {
            k: v for k, v in self.tracks.items() if stamp - v[0] <= self.max_gap
        }
        return x[:3].copy(), True
