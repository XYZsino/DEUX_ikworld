"""Additive reader-axis yaw adjustment; preserves core lookup hash inputs."""
from contextlib import contextmanager
import numpy as np
from scipy.spatial.transform import Rotation


def rotate_target(target,extra_yaw_deg):
    extra=float(extra_yaw_deg)
    if not np.isfinite(extra) or not -45.<=extra<=45.:raise ValueError('reader extra yaw must be finite and within +/-45deg')
    result=np.asarray(target,float).copy()
    if result.shape!=(4,4) or not np.isfinite(result).all():raise ValueError('finite reader target required')
    result[:3,:3]=Rotation.from_euler('z',extra,degrees=True).as_matrix()@result[:3,:3]
    return result


class ReaderYawSolver:
    def __init__(self,solver,extra):self.solver=solver;self.extra=extra
    def __getattr__(self,name):return getattr(self.solver,name)
    def solve_reader_axis(self,side,target,offset,seed):
        if side!='left':raise ValueError('reader yaw adjustment is left-arm only')
        return self.solver.solve_reader_axis(side,rotate_target(target,self.extra),offset,seed)


@contextmanager
def adjusted_reader(planner,stage,extra_yaw_deg):
    extra=float(extra_yaw_deg);rotate_target(np.eye(4),extra)
    if stage!='AlignBarcode' or extra==0.:
        yield;return
    if not planner.config.get('barcode_position_only',False):raise ValueError('reader extra yaw requires position-only barcode policy')
    original=getattr(planner.robot,'ik_solver',None)
    if not hasattr(original,'solve_reader_axis'):raise ValueError('reader extra yaw requires reader-axis IK backend')
    planner.robot.ik_solver=ReaderYawSolver(original,extra)
    try:yield
    finally:planner.robot.ik_solver=original
