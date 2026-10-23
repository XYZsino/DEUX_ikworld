"""ROS-independent validation/conversion; all timestamps are integer nanoseconds."""
from collections import deque
import numpy as np

ARM_NAMES = tuple(f'arm_{s}_joint{i}' for s in ('l', 'r') for i in range(1, 8))
HAND_PARTS = ('thumb_joint_1', 'thumb_joint_2', 'thumb_joint_3', 'index_joint_1',
              'index_joint_2', 'third_joint_1', 'third_joint_2')
JOINT_NAMES = tuple(n for side in ('left', 'right') for n in
                    (*[f'arm_{side[0]}_joint{i}' for i in range(1, 8)],
                     *[f'{side}_hand_{part}' for part in HAND_PARTS]))


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def decode_image(msg, channels, dtype):
    """Respect ROS row padding and endianness; reject truncated buffers."""
    dt = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    row_bytes = msg.width * channels * dt.itemsize
    if msg.step < row_bytes or len(msg.data) != msg.step * msg.height:
        raise ValueError('invalid image step/data length')
    return np.ndarray((msg.height, msg.width, channels), dtype=dt,
                      buffer=bytes(msg.data), strides=(msg.step, channels * dt.itemsize,
                                                     dt.itemsize)).copy()


def normalize_rgb(msg):
    formats = {'rgb8': (3, [0, 1, 2]), 'bgr8': (3, [2, 1, 0]),
               'rgba8': (4, [0, 1, 2]), 'bgra8': (4, [2, 1, 0])}
    if msg.encoding not in formats:
        raise ValueError(f'unsupported RGB encoding: {msg.encoding}')
    channels, order = formats[msg.encoding]
    return np.ascontiguousarray(decode_image(msg, channels, 'u1')[:, :, order])


def normalize_depth(msg):
    if msg.encoding == '32FC1':
        depth = decode_image(msg, 1, 'f4')[:, :, 0].astype(np.float32)
    elif msg.encoding in ('16UC1', 'mono16'):
        # ZED OpenNI mode: MONO16 millimetres, not grayscale intensity.
        depth = decode_image(msg, 1, 'u2')[:, :, 0].astype(np.float32) * .001
    else:
        raise ValueError(f'unsupported depth encoding: {msg.encoding}')
    valid = np.isfinite(depth) & (depth > 0)
    depth[~valid] = np.nan
    return np.ascontiguousarray(depth, dtype='<f4'), float(valid.mean())


def depth_view(msg):
    """Read-only strided ROS buffer view; apply units only inside selected ROIs."""
    if msg.encoding == '32FC1':
        dtype, scale = 'f4', 1.0
    elif msg.encoding in ('16UC1', 'mono16'):
        dtype, scale = 'u2', .001
    else:
        raise ValueError(f'unsupported depth encoding: {msg.encoding}')
    dt = np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    if msg.step < msg.width * dt.itemsize or len(msg.data) != msg.step * msg.height:
        raise ValueError('invalid image step/data length')
    view = np.ndarray((msg.height, msg.width), dtype=dt, buffer=memoryview(msg.data),
                      strides=(msg.step, dt.itemsize))
    view.setflags(write=False)
    return view, scale


def validate_bundle(rgb, depth, rgb_info, depth_info, width, height, slop_ns):
    messages = (rgb, depth, rgb_info, depth_info)
    if any((m.width, m.height) != (width, height) for m in messages):
        raise ValueError(f'all images/CameraInfo must be {width}x{height}; no implicit resize')
    frames = {m.header.frame_id for m in messages}
    if len(frames) != 1 or not next(iter(frames)):
        raise ValueError('registered RGB/depth/CameraInfo optical frame mismatch')
    times = [stamp_ns(m.header) for m in messages]
    if min(times) < 0 or max(times) - min(times) > slop_ns:
        raise ValueError('RGB-D/CameraInfo timestamp skew')
    for info in (rgb_info, depth_info):
        k = np.asarray(info.k).reshape(3, 3)
        proj = np.asarray(info.p).reshape(3, 4)
        if (not np.isfinite(k).all() or not np.isfinite(proj).all()
                or min(k[0, 0], k[1, 1], proj[0, 0], proj[1, 1]) <= 0
                or not np.allclose(k[2], [0, 0, 1])):
            raise ValueError('invalid or uncalibrated CameraInfo')
        if (info.binning_x not in (0, 1) or info.binning_y not in (0, 1)
                or info.roi.x_offset or info.roi.y_offset
                or info.roi.width not in (0, width) or info.roi.height not in (0, height)):
            raise ValueError('nontrivial CameraInfo ROI/binning requires explicit upstream rectification')
    if not np.allclose(rgb_info.p, depth_info.p, atol=1e-5):
        raise ValueError('registered depth projection differs from RGB')


class History:
    """Select nearest causal measurement; never relabel an old sample as new."""
    def __init__(self, capacity=120):
        self.samples = deque(maxlen=capacity)

    def add(self, msg):
        self.samples.append(msg)

    def at(self, timestamp, max_age_ns):
        candidates = [m for m in self.samples if 0 <= timestamp-stamp_ns(m.header) <= max_age_ns]
        return max(candidates, key=lambda m: stamp_ns(m.header)) if candidates else None

    def clear(self):
        self.samples.clear()


def joint_values(msg, names):
    if len(msg.name) != len(msg.position) or len(set(msg.name)) != len(msg.name):
        raise ValueError('malformed/duplicate joint names')
    values = dict(zip(msg.name, msg.position))
    result = np.array([values[name] for name in names], dtype=float)
    if not np.isfinite(result).all():
        raise ValueError('nonfinite joint measurement')
    return result.tolist()


def validate_objects(msg, base_frame):
    if msg.header.frame_id != base_frame:
        raise ValueError('object poses must already be in configured robot base frame')
    ids = [d.track_id for d in msg.detections]
    if len(set(ids)) != len(ids) or any(not i for i in ids):
        raise ValueError('missing or duplicate object identity')
    for detection in msg.detections:
        pose = detection.pose.pose
        q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z,
                      pose.orientation.w])
        values = [pose.position.x, pose.position.y, pose.position.z,
                  detection.confidence, *detection.pose.covariance, *q]
        if (not np.isfinite(values).all() or not 0 <= detection.confidence <= 1
                or not np.isclose(np.linalg.norm(q), 1, atol=1e-3)):
            raise ValueError('invalid object pose/confidence/covariance')
        cov = np.asarray(detection.pose.covariance).reshape(6, 6)
        if not np.allclose(cov, cov.T) or np.linalg.eigvalsh(cov).min() < -1e-9:
            raise ValueError('object covariance must be symmetric positive semidefinite')
