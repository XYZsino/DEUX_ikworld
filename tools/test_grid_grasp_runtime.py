"""Pure linkage regression: timestamps, transforms, speed and frozen-cell guards."""
import unittest
import numpy as np
import yaml
import json
import tempfile
from pathlib import Path
from grid_grasp_runtime import GridLibrary, ROOT, sha


class LinkTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = ROOT/'src/deux_task_bringup/config/installation.yaml'
        config = yaml.safe_load(path.read_text())
        config['robot']['urdf_path'] = str((path.parent/config['robot']['urdf_path']).resolve())
        cls.lib = GridLibrary(config)

    def observation(self, age=0.):
        obj = np.eye(4); obj[:3, 3] = [.48, .48+.09*age, .97]
        return obj, np.array([0., -.09, 0., 0.]), 100.-age, 100.

    def test_latency_compensation_and_absolute_dispatch(self):
        a = self.lib.select(*self.observation())
        b = self.lib.select(*self.observation(.2))
        self.assertEqual(a['cell_id'], b['cell_id'])
        self.assertAlmostEqual(a['start_at_s'], b['start_at_s'])
        self.assertAlmostEqual(a['contact_at_s']-a['start_at_s'], 3.)
        g = a['contact_xyz_m']; self.assertAlmostEqual(g[1]+.27, .48-.09*a['start_in_s'])

    def test_object_to_grasp_translation_is_applied(self):
        offset = self.lib.offset.copy()
        try:
            self.lib.offset = offset.copy(); self.lib.offset[0, 3] = .01
            obj, v, stamp, now = self.observation(); obj[0, 3] -= .01
            chosen = self.lib.select(obj, v, stamp, now)
            self.assertAlmostEqual(chosen['grasp_now_m'][0], .48)
        finally: self.lib.offset = offset

    def test_stale_prediction_refused(self):
        with self.assertRaisesRegex(ValueError, 'age'): self.lib.select(*self.observation(.51))

    def test_known_belt_speed_drives_timing_instead_of_noisy_estimated_velocity(self):
        obj, v, stamp, now = self.observation(); v[1] = .09
        chosen = self.lib.select(obj, v, stamp, now)
        self.assertEqual(chosen['velocity_m_s'], [0., -.09, 0.])
        self.assertAlmostEqual(chosen['contact_in_s'], (.48-chosen['contact_xyz_m'][1])/.09)

    def test_model_center_z_is_distinct_from_user_fixed_grasp_plane(self):
        obj, v, stamp, now = self.observation(); obj[2, 3] = .927; v[2] = .02
        chosen = self.lib.select(obj, v, stamp, now)
        self.assertAlmostEqual(chosen['grasp_now_m'][2], .97)
        self.assertAlmostEqual(chosen['contact_xyz_m'][2], .97)

    def test_xy_selection_ignores_noisy_object_yaw(self):
        from scipy.spatial.transform import Rotation
        obj, v, stamp, now = self.observation(); obj[:3, :3] = Rotation.from_euler('z', 120, degrees=True).as_matrix()
        chosen = self.lib.select(obj, v, stamp, now)
        self.assertEqual(chosen['contact_xyz_m'][0], .48)

    def test_selected_cell_not_replaced_when_prediction_drifts(self):
        obj, v, stamp, now = self.observation(); chosen = self.lib.select(obj, v, stamp, now)
        obj[0, 3] += .04
        with self.assertRaisesRegex(ValueError, 'departed'): self.lib.live_error(chosen, obj, v, stamp, now)

    def test_stationary_left_requires_matching_audit(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary)/'left.json'
            audit = dict(left_arm_rad=[0.]*7, manifest_sha256=sha(ROOT/'artifacts/right_grasp_grid_fast/manifest.json'),
                         validated_cells=list(self.lib.entries))
            path.write_text(json.dumps(audit))
            frozen = GridLibrary(self.lib.config, stationary_left_file=path)
            np.testing.assert_allclose(frozen.ready[:7], 0.)
            np.testing.assert_allclose(frozen.ready[14:21], self.lib.ready[14:21])
            audit['manifest_sha256'] = 'wrong'; path.write_text(json.dumps(audit))
            with self.assertRaisesRegex(ValueError, 'another library'):
                GridLibrary(self.lib.config, stationary_left_file=path)

    def test_xy_tolerance_includes_30mm_boundary_but_rejects_larger_error(self):
        obj, v, stamp, now = self.observation(); obj[0, 3] = .60
        chosen = self.lib.select(obj, v, stamp, now)
        self.assertAlmostEqual(chosen['lateral_error_m'], .03)
        obj[0, 3] += .0001
        with self.assertRaisesRegex(ValueError, 'no cell'):
            self.lib.select(obj, v, stamp, now)

    def test_live_xy_tolerance_is_euclidean_30mm(self):
        obj, v, stamp, now = self.observation(); chosen = self.lib.select(obj, v, stamp, now)
        baseline = np.array(chosen['contact_xyz_m'])+self.lib.velocity*(now-chosen['contact_at_s'])
        obj[:3, 3] = baseline; obj[:2, 3] += [.021, .021]
        self.lib.live_error(chosen, obj, v, stamp, now)
        obj[:3, 3] = baseline; obj[:2, 3] += [.022, .022]
        with self.assertRaisesRegex(ValueError, '30mm'):
            self.lib.live_error(chosen, obj, v, stamp, now)


if __name__ == '__main__': unittest.main()
