"""Conservative visible-surface association after a grasp occlusion.

This selects an observed candidate, never invents tracker identity continuity.
"""
import numpy as np


def held_candidate(candidates, hand_xyz, previous_surface_z):
    if previous_surface_z is None:return None
    matches=[]
    for item in candidates:
        if not item.bag_visible or not item.surface_center_valid:continue
        p=item.surface_center
        xyz=np.array([p.x,p.y,p.z],dtype=float)
        if not np.isfinite(xyz).all():continue
        if p.z < previous_surface_z+.04:continue
        if np.linalg.norm(xyz-np.asarray(hand_xyz))>.20:continue
        matches.append(item)
    return matches[0] if len(matches)==1 else None
