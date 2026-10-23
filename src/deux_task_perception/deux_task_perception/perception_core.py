"""Geometry and bounded asynchronous work, independent of ROS and YOLO."""
from concurrent.futures import ThreadPoolExecutor
from hashlib import sha256
from pathlib import Path
from uuid import uuid4

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation


def rot(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.]])


def downsample(points, limit=1600):
    if not len(points):
        return points
    _, indices = np.unique(np.floor(points / .003).astype(int), axis=0,
                           return_index=True)
    points = points[np.sort(indices)]
    if len(points) > limit:
        points = points[np.linspace(0, len(points) - 1, limit).astype(int)]
    return points


def solve_yaw(model, observed, weights):
    weights = weights / weights.sum()
    mc = (model * weights[:, None]).sum(0)
    oc = (observed * weights[:, None]).sum(0)
    a, b = model - mc, observed - oc
    theta = np.arctan2(np.sum(weights * (a[:, 0]*b[:, 1] - a[:, 1]*b[:, 0])),
                       np.sum(weights * (a[:, 0]*b[:, 0] + a[:, 1]*b[:, 1])))
    return theta, oc - mc @ rot(theta).T


def load_model(path, origin, count=12000):
    """Sample an OBJ once; no external cache or implicit origin conversion."""
    if origin not in ('mesh', 'bounds_center'):
        raise ValueError('model_origin must be mesh or bounds_center')
    raw = Path(path).read_bytes()
    vertices, faces = [], []
    for line in raw.decode().splitlines():
        fields = line.split()
        if not fields:
            continue
        if fields[0] == 'v':
            vertices.append([float(v) for v in fields[1:4]])
        elif fields[0] == 'f':
            face = [int(v.split('/')[0]) for v in fields[1:]]
            if any(v == 0 for v in face):
                raise ValueError('OBJ vertex index zero')
            face = [v-1 if v > 0 else len(vertices)+v for v in face]
            faces.extend((face[0], face[i], face[i+1]) for i in range(1, len(face)-1))
    v = np.asarray(vertices, dtype=float)
    if v.ndim != 2 or v.shape[1] != 3 or not np.isfinite(v).all() or not faces:
        raise ValueError('invalid mesh')
    bounds = np.array([v.min(0), v.max(0)])
    offset = bounds.mean(0) if origin == 'bounds_center' else np.zeros(3)
    triangles = (v-offset)[np.asarray(faces)]
    cross = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    area = np.linalg.norm(cross, axis=1)
    if not np.isfinite(area).all() or area.sum() <= 0:
        raise ValueError('degenerate mesh')
    rng = np.random.default_rng(42)
    indices = rng.choice(len(triangles), count, p=area/area.sum())
    uv = rng.random((count, 2))
    uv[:, 0] = np.sqrt(uv[:, 0])
    tri = triangles[indices]
    points = ((1-uv[:, 0, None])*tri[:, 0] +
              (uv[:, 0]*(1-uv[:, 1]))[:, None]*tri[:, 1] +
              (uv[:, 0]*uv[:, 1])[:, None]*tri[:, 2])
    return points, cross[indices]/area[indices, None], f'obj-sha256:{sha256(raw).hexdigest()}:{origin}:metres'


def transform_matrix(translation, quaternion):
    t, q = np.asarray(translation), np.asarray(quaternion)
    if not np.isfinite(t).all() or not np.isfinite(q).all() or not np.isclose(np.linalg.norm(q), 1, atol=1e-3):
        raise ValueError('invalid calibrated transform')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(q).as_matrix()
    matrix[:3, 3] = t
    return matrix


def roi_points(depth, projection, box, reference_from_camera, min_height, max_distance):
    """Rectified registered depth: use CameraInfo.P, not raw-camera K."""
    p = np.asarray(projection, dtype=float).reshape(3, 4)
    box = np.asarray(box, dtype=float)
    if (not np.isfinite(p).all() or p[0, 0] <= 0 or p[1, 1] <= 0 or
            not np.allclose(p[2], [0, 0, 1, 0]) or
            not np.allclose([p[0, 1], p[1, 0]], 0)):
        raise ValueError('unsupported rectified projection')
    if box.shape != (4,) or not np.isfinite(box).all() or np.any(box[2:] <= box[:2]):
        raise ValueError('invalid bounding box')
    h, w = depth.shape
    x0, y0 = np.maximum(np.ceil(box[:2]), 0).astype(int)
    x1, y1 = np.minimum(np.floor(box[2:])+1, [w, h]).astype(int)
    if x1 <= x0 or y1 <= y0:
        return np.empty((0, 3))
    v, u = np.mgrid[y0:y1, x0:x1]
    z = depth[y0:y1, x0:x1]
    valid = np.isfinite(z) & (z > 0)
    z, u, v = z[valid], u[valid], v[valid]
    camera = np.column_stack(((u-p[0, 2])*z/p[0, 0]-p[0, 3]/p[0, 0],
                              (v-p[1, 2])*z/p[1, 1]-p[1, 3]/p[1, 1], z))
    reference = camera @ reference_from_camera[:3, :3].T + reference_from_camera[:3, 3]
    return reference[(np.linalg.norm(camera, axis=1) <= max_distance) &
                     (reference[:, 2] > min_height)]


class LatestWorker:
    """One active job plus one replaceable pending frame; executor owns all state."""
    def __init__(self, function):
        self.function = function
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.pending = None
        self.dropped = 0

    def submit(self, value):
        if self.pending is not None:
            self.dropped += 1
        self.pending = value

    def poll(self):
        completed = self.future if self.future is not None and self.future.done() else None
        if completed is not None:
            self.future = None
        if self.future is None and self.pending is not None:
            self.future = self.pool.submit(self.function, self.pending)
            self.pending = None
        return completed

    def close(self):
        self.pending = None
        self.pool.shutdown(wait=True, cancel_futures=True)


class Tracker:
    """Gated nearest-position association; IDs are not guaranteed through crossings."""
    def __init__(self, gate=.08, max_age=.6):
        self.gate, self.max_age = gate, max_age
        self.prefix = uuid4().hex[:12]
        self.counter = 0
        self.tracks = {}

    def update(self, positions, stamp):
        self.tracks = {k: v for k, v in self.tracks.items() if 0 <= stamp-v[1] <= self.max_age}
        keys = list(self.tracks)
        ids = [None]*len(positions)
        if keys and len(positions):
            distances = np.linalg.norm(np.asarray([self.tracks[k][0] for k in keys])[:, None] -
                                       np.asarray(positions)[None], axis=2)
            rows, cols = linear_sum_assignment(np.where(distances <= self.gate, distances, 1e6))
            for row, col in zip(rows, cols):
                if distances[row, col] <= self.gate:
                    ids[col] = keys[row]
        for i, position in enumerate(positions):
            if ids[i] is None:
                ids[i] = f'{self.prefix}-{self.counter}'
                self.counter += 1
            self.tracks[ids[i]] = (np.array(position), stamp)
        return ids
