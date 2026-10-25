"""Conservative RGB-D label surface estimation; no decoding or bag orientation inference."""
import numpy as np
from .perception_core import roi_points


def surface(depth, projection, box, max_distance=1.5):
    x0, y0, x1, y1 = box
    # Inner patch avoids bbox background and printed label boundaries.
    inner = [x0+.2*(x1-x0), y0+.2*(y1-y0), x1-.2*(x1-x0), y1-.2*(y1-y0)]
    points = roi_points(depth, projection, inner, np.eye(4), 0., max_distance)
    area = max(1, (int(inner[2])-int(np.ceil(inner[0]))+1)*(int(inner[3])-int(np.ceil(inner[1]))+1))
    fraction = min(1., len(points)/area)
    if len(points) < 30 or fraction < .7:
        raise ValueError('insufficient label depth')
    median = np.median(points[:, 2])
    points = points[abs(points[:, 2]-median) < .015]
    if len(points) < 30 or len(points)/area < .7:
        raise ValueError('mixed foreground/background label depth')
    center = np.mean(points, axis=0)
    _, singular, axes = np.linalg.svd(points-center, full_matrices=False)
    normal = axes[-1]
    rmse = float(np.sqrt(np.mean(((points-center)@normal)**2)))
    if rmse > .003 or singular[1]/np.sqrt(len(points)) < .002:
        raise ValueError('label plane ill-conditioned')
    if normal @ center > 0: normal = -normal
    x = np.array([1., 0., 0.]); x -= normal*(normal@x)
    if np.linalg.norm(x) < .2: raise ValueError('grazing label surface')
    x /= np.linalg.norm(x)
    pose = np.eye(4); pose[:3, :3] = np.column_stack((x, np.cross(normal, x), normal))
    pose[:3, 3] = center
    return pose, rmse, fraction


def associated_bag(label, bags, camera_target, projection):
    """Require one enclosing bag and one requested target projection inside it."""
    cx, cy = (np.array(label[:2])+label[2:])/2
    containers = [b for b in bags if b[0] <= cx <= b[2] and b[1] <= cy <= b[3]]
    if len(containers) != 1 or camera_target[2] <= 0:
        raise ValueError('ambiguous label/bag association')
    p = np.array(projection).reshape(3, 4)
    uv = p @ np.r_[camera_target, 1.]; uv = uv[:2]/uv[2]
    b = containers[0]
    if not (b[0] <= uv[0] <= b[2] and b[1] <= uv[1] <= b[3]):
        raise ValueError('label belongs to a different target')
