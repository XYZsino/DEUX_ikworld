"""Position-only barcode quality and reader look-at target."""
import numpy as np
from scipy.spatial.transform import Rotation


def label_quality(msg, config):
    if config.get('barcode_position_only', False):
        return (not msg.orientation_valid and np.isfinite(msg.valid_fraction)
                and 0 < msg.valid_fraction <= 1.)
    return (msg.orientation_valid and np.isfinite([msg.plane_rmse_m,msg.valid_fraction]).all()
            and 0 <= msg.plane_rmse_m <= .003 and .7 <= msg.valid_fraction <= 1.)


def reader_look_at(current, point, distance, aim_offset_base=(0., 0., 0.), yaw_offset_deg=0.):
    current=np.asarray(current,dtype=float); point=np.asarray(point,dtype=float)
    offset=np.asarray(aim_offset_base,dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError('invalid barcode aim offset in base frame')
    point=point+offset
    ray=point-current[:3,3]; length=np.linalg.norm(ray)
    if not np.isfinite(ray).all() or length < 1e-6 or not np.isfinite(distance) or distance <= 0:
        raise ValueError('invalid barcode reader look-at geometry')
    z=ray/length; old_z=current[:3,2]
    cross=np.cross(old_z,z); sine=np.linalg.norm(cross); cosine=np.clip(old_z@z,-1.,1.)
    if sine > 1e-9:
        delta=Rotation.from_rotvec(cross/sine*np.arctan2(sine,cosine)).as_matrix()
    elif cosine < 0:
        delta=Rotation.from_rotvec(current[:3,0]*np.pi).as_matrix()
    else:
        delta=np.eye(3)
    target=current.copy(); target[:3,:3]=delta@current[:3,:3]
    target[:3,3]=point-distance*z
    if not np.isfinite(yaw_offset_deg):
        raise ValueError('invalid reader base yaw offset')
    # Rotate direction about base +Z without changing the reader origin.
    target[:3,:3]=Rotation.from_euler('z',yaw_offset_deg,degrees=True).as_matrix()@target[:3,:3]
    return target
