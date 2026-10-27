"""Acquired depth surface bounds and image projection, shared by BT evidence/UI."""
from itertools import product
import numpy as np
from .perception_core import masked_depth


def project_bounds(lo, hi, base_from_camera, projection):
    xyz = np.array(list(product(*zip(lo, hi))))
    bc = np.asarray(base_from_camera)
    optical = (xyz-bc[:3, 3]) @ bc[:3, :3]
    if not np.isfinite(optical).all() or np.any(optical[:, 2] <= .03):
        raise ValueError('3D box crosses camera plane')
    p = np.asarray(projection).reshape(3, 4)
    projected = np.c_[optical, np.ones(8)] @ p.T
    pixels = projected[:, :2] / projected[:, 2, None]
    if not np.isfinite(pixels).all():
        raise ValueError('invalid box projection')
    return pixels


def workspace_roi(region, base_from_camera, projection, width, height, vertical_fraction=None):
    pixels = project_bounds(region['min'], region['max'], base_from_camera, projection)
    lo = np.maximum(pixels.min(axis=0), [0., 0.])
    hi = np.minimum(pixels.max(axis=0), [float(width), float(height)])
    if vertical_fraction is not None:
        y=np.asarray(vertical_fraction,dtype=float)
        if y.shape!=(2,) or not np.isfinite(y).all() or not 0<=y[0]<y[1]<=1:
            raise ValueError('invalid selection ROI vertical fraction')
        lo[1],hi[1]=y*height
    if np.any(hi-lo < 1.):
        raise ValueError('selection workspace outside RGB')
    return np.r_[lo, hi]


def surface_bounds(depth, projection, polygon, base_from_camera):
    """Robust visible-surface AABB, not a complete object model or ICP pose."""
    selected = masked_depth(depth, polygon)
    ys, xs = np.where(np.isfinite(selected) & (selected > .05) & (selected < 3.))
    stride = max(1, len(xs)//6000)
    ys, xs = ys[::stride], xs[::stride]
    if len(xs) < 40:
        raise ValueError('유효한 대상 깊이 부족')
    z = selected[ys, xs]
    median = np.median(z); mad = np.median(np.abs(z-median))
    keep = np.abs(z-median) <= min(.08, max(.015, 3.*1.4826*mad))
    z, xs, ys = z[keep], xs[keep], ys[keep]
    if len(xs) < 40:
        raise ValueError('일관된 대상 깊이 부족')
    p = np.asarray(projection).reshape(3, 4)
    if (not np.isfinite(p).all() or p[0, 0] <= 0 or p[1, 1] <= 0
            or not np.allclose(p[2], [0, 0, 1, 0])
            or not np.allclose([p[0, 1], p[1, 0]], 0)):
        raise ValueError('지원하지 않는 투영 행렬')
    points = np.column_stack(((xs-p[0, 2])*z/p[0, 0]-p[0, 3]/p[0, 0],
                             (ys-p[1, 2])*z/p[1, 1]-p[1, 3]/p[1, 1], z))
    bc = np.asarray(base_from_camera)
    world = points @ bc[:3, :3].T + bc[:3, 3]
    lo, hi = np.percentile(world, [2, 98], axis=0)
    extent = hi-lo
    if not np.isfinite(extent).all() or np.any(extent > .6) or np.max(extent) < .01:
        raise ValueError('대상 3D 범위 불안정')
    return lo, hi
