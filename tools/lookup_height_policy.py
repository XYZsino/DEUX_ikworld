"""Reuse offline replay clearance for the same installed height snapshot.

There is no live conveyor/lift height sensor in this stack. Lift motion invalidates
camera calibration independently; never treat an invalid snapshot as a new height.
"""
import json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]

def heights(config):
    g=config['thor_geometry'];d=config['thor_development']
    surface=json.loads((ROOT/'calibration/conveyor_visible_surface.json').read_text());plane=surface['plane'];normal=np.asarray(plane['normal'],float)
    values=dict(shoulder_floor_m=float(g['shoulder_height_from_floor_m']),lift_origin_base_m=float(g['lift_origin_z_in_base_m']),equivalent_lift_joint_m=float(g['equivalent_prismatic_lift_joint_m']),camera_floor_m=float(g['camera_height_from_floor_m']),conveyor_floor_m=float(d['conveyor_height_floor_m']),conveyor_plane_z_at_base_origin_m=float(-plane['d']/normal[2]))
    if not np.isfinite(list(values.values())).all():raise ValueError('finite installation heights required')
    return values

def same_heights(config,baseline,tolerance_m=.001):
    current=heights(config)
    if set(current)!=set(baseline):return False,current
    return all(abs(current[k]-float(baseline[k]))<=tolerance_m for k in current),current


def load_paths(library,config,progress=None,left_target=None,profile='legacy'):
    from lookup_lift_scan import compile_path,compile_library
    if (ROOT/'.setup/camera-calibration-lift-invalid.json').exists():raise ValueError('Lift snapshot invalidated; update installed heights/calibration before replay')
    if profile=='via_preserved_fast_scan':
        from lookup_fast_scan import load_audited
        return load_audited(library,config,left_target)
    if profile!='legacy':raise ValueError('unknown scan timing profile')
    record=json.loads((ROOT/'artifacts/dataset-barcode-pose/replay-height-baseline.json').read_text())
    matches,current=same_heights(config,record['heights'])
    if not matches:
        if progress:progress(0,0,0)
        return compile_library(library,config,progress,left_target=left_target)
    report=json.loads((ROOT/'artifacts/right-high-scan-waypoint/library-validation.json').read_text())
    rejected=dict(report['rejected']);accepted={}
    for key in library.entries:
        if key in rejected:continue
        accepted[key]=compile_path(library,key,config,left_target=left_target,reuse_clearance=float(report['minimum_clearance_m']))
    return accepted,rejected
