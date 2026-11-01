"""Extend the existing stationary-left audit to all accepted stored cells. No ROS commands."""
import json
import time
import numpy as np
import yaml
from grid_grasp_runtime import GridLibrary, ROOT, sha
from upper_collision_batch import checked_clearances


def main():
    path = ROOT/'src/deux_task_bringup/config/installation.yaml'
    config = yaml.safe_load(path.read_text()); config['robot']['urdf_path'] = str((path.parent/config['robot']['urdf_path']).resolve())
    file = ROOT/'.setup/grid-grasp-test/stationary-left.json'; audit = json.loads(file.read_text())
    if audit['manifest_sha256'] != sha(ROOT/'artifacts/right_grasp_grid_fast/manifest.json'): raise ValueError('audit/library mismatch')
    library = GridLibrary(config, recommended=False); validated = {}
    left = np.asarray(audit['left_arm_rad'], float)
    if left.shape != (7,) or not np.isfinite(left).all(): raise ValueError('invalid left snapshot')
    for key in library.entries:
        tt = np.linspace(0., 5., 501); q = np.tile(library.ready, (len(tt), 1))
        q[:, :7] = left; q[:, 14:21] = library.curves[key](tt)
        validated[key] = float(np.min(checked_clearances(library.robot, q, config['obstacles'], config['collision_margin'])))
    audit.update(validated_cells=validated, full_library_audited_at_s=time.time())
    temporary = file.with_suffix('.tmp'); temporary.write_text(json.dumps(audit, indent=2)); temporary.replace(file)
    print(json.dumps(dict(validated_cells=len(validated), left_arm_rad=left.tolist(), no_motion_commands=True)))


if __name__ == '__main__': main()
