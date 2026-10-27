import threading
import time
import numpy as np
import pytest
from deux_task_perception.perception_core import (
    LatestWorker, Tracker, load_model, roi_points, rot, transform_matrix)
from deux_task_perception.yaw_icp import fit_icp


def test_rectified_projection_and_reference_height():
    depth = np.array([[1., np.nan], [0., 1.]])
    p = [2., 0., 0., .2, 0., 2., 0., .4, 0., 0., 1., 0.]
    transform = np.eye(4)
    transform[2, 3] = -0.8
    points = roi_points(depth, p, [-10., -10., 10., 10.], transform, .1, 1.5)
    np.testing.assert_allclose(points, [[-.1, -.2, .2], [.4, .3, .2]])
    assert len(roi_points(depth, p, [0, 0, 2, 2], transform, .3, 1.5)) == 0
    assert len(roi_points(depth, p, [10, 10, 20, 20], transform, .1, 1.5)) == 0
    with pytest.raises(ValueError):
        roi_points(depth, p, [0, 0, float('nan'), 1], transform, .1, 1.5)


def test_model_origin_hash_and_no_unversioned_cache(tmp_path):
    path = tmp_path / 'triangle.obj'
    path.write_text('v 0 0 0\nv 2 0 0\nv 0 4 2\nf 1 2 3\n')
    mesh, normals, identity = load_model(path, 'mesh', 100)
    centered, _, centered_id = load_model(path, 'bounds_center', 100)
    np.testing.assert_allclose(mesh-centered, np.tile([1, 2, 1], (100, 1)))
    np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1)
    assert identity != centered_id
    path.write_text(path.read_text().replace('v 2 0 0', 'v 3 0 0'))
    assert load_model(path, 'mesh', 100)[2] != identity
    with pytest.raises(ValueError):
        load_model(path, '')


def test_latest_worker_replaces_pending_and_propagates_errors():
    ready, release = threading.Event(), threading.Event()
    calls = []
    def work(value):
        calls.append(value)
        if value == 1:
            ready.set()
            assert release.wait(3)
        if value == 4:
            raise ValueError('synthetic failure')
        return value
    worker = LatestWorker(work)
    def result():
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            completed = worker.poll()
            if completed is not None:
                return completed.result()
            time.sleep(.001)
        pytest.fail('worker timeout')
    try:
        worker.submit(1)
        worker.poll()
        assert ready.wait(1)
        worker.submit(2)
        worker.submit(3)
        release.set()
        assert result() == 1
        assert result() == 3
        assert calls == [1, 3] and worker.dropped == 1
        worker.submit(4)
        with pytest.raises(ValueError, match='synthetic failure'):
            result()
    finally:
        release.set()
        worker.close()


def test_tracker_reordering_distance_expiry_and_clock_reset():
    tracker = Tracker(.08, .6)
    first = tracker.update([[0, 0, 0], [1, 0, 0]], 10.)
    assert tracker.update([[1.01, 0, 0], [.01, 0, 0]], 10.1) == first[::-1]
    assert tracker.update([[2, 0, 0]], 10.2)[0] not in first
    assert tracker.update([[0, 0, 0]], 11.)[0] not in first
    assert tracker.update([[1, 0, 0]], 1.)[0] not in first


def test_yaw_icp_recovers_asymmetric_synthetic_cloud():
    rng = np.random.default_rng(42)
    model = rng.uniform([-.06, -.04, -.08], [.06, .04, .08], (1200, 3))
    normals = model / np.linalg.norm(model, axis=1)[:, None]
    translation = np.array([.2, -.1, .4])
    theta = np.pi/6
    observed = model[::3] @ rot(theta).T + translation
    result = fit_icp(observed, model, normals, np.array([0., 0., 1.]))
    assert result['status'] == 'fitted'
    np.testing.assert_allclose(result['T_reference_model'],
                               transform_matrix(translation, [0, 0, np.sin(theta/2), np.cos(theta/2)]), atol=.002)
    assert result['trimmed_rmse_m'] < .002
    assert result['target_coverage_10mm'] > .95
    assert fit_icp(observed[:10], model, normals, np.zeros(3))['status'] == 'insufficient_depth'


@pytest.mark.parametrize('limit', [80, 400, 600, 1600])
def test_coarse_subsampling_retains_full_cloud_metrics(limit):
    from scipy.spatial import cKDTree
    from deux_task_perception.perception_core import downsample
    rng = np.random.default_rng(9)
    model = rng.uniform([-.08, -.04, -.03], [.08, .04, .03], (3600, 3))
    normals = model / np.linalg.norm(model, axis=1)[:, None]
    observed = model[::3] @ rot(np.pi/6).T + [.2, -.1, .4]
    result = fit_icp(observed, model, normals, np.array([0., 0., 1.]),
                     initial_angles=12, yaw_span_deg=180., coarse_point_limit=limit)
    q = downsample(observed)
    assert result['coarse_fit_points'] == min(limit, len(q))
    assert len(result['multistart']) == 12
    # Independently recompute published metrics from every fine-resolution point.
    dist = cKDTree(model[::3]).query((q-np.array(result['translation'])) @ rot(result['theta']))[0]
    assert result['target_coverage_10mm'] == pytest.approx(np.mean(dist < .01))
    assert result['trimmed_rmse_m'] == pytest.approx(np.sqrt(np.mean(np.sort(dist)[:int(.8*len(dist))]**2)))
    np.testing.assert_allclose(result['translation'], [.2, -.1, .4], atol=.003)
    assert abs(result['theta']-np.pi/6) < .03


@pytest.mark.parametrize('kwargs', [{'coarse_point_limit': 39}, {'coarse_point_limit': 4.5},
                                  {'initial_angles': 0}, {'yaw_span_deg': float('nan')}])
def test_invalid_icp_sampling_configuration(kwargs):
    with pytest.raises(ValueError, match='invalid ICP'):
        fit_icp(np.empty((0, 3)), np.empty((0, 3)), np.empty((0, 3)), np.zeros(3), **kwargs)


def test_independent_coarse_voxel_cloud_and_full_resolution_metrics(monkeypatch):
    from scipy.spatial import cKDTree
    from deux_task_perception import yaw_icp
    from deux_task_perception.perception_core import downsample, voxel_downsample
    rng = np.random.default_rng(9)
    model = rng.uniform([-.08, -.04, -.03], [.08, .04, .03], (9000, 3))
    normals = model / np.linalg.norm(model, axis=1)[:, None]
    observed = model[::3] @ rot(np.pi/6).T + [.2, -.1, .4]
    calls = []
    def capture(points, size):
        calls.append(points)
        return voxel_downsample(points, size)
    monkeypatch.setattr(yaw_icp, 'voxel_downsample', capture)
    result = fit_icp(observed, model, normals, np.array([0., 0., 1.]),
                     initial_angles=12, yaw_span_deg=180., coarse_voxel_size_m=.006)
    assert len(calls) == 1 and calls[0] is observed
    expected = len(voxel_downsample(observed, .006))
    assert result['coarse_voxel_points'] == expected
    assert result['coarse_fit_points'] == min(1600, expected)
    assert not result['coarse_fallback']
    q = downsample(observed)
    dist = cKDTree(model[::3]).query((q-result['translation']) @ rot(result['theta']))[0]
    assert result['fit_points'] == len(q)
    assert result['target_coverage_10mm'] == pytest.approx(np.mean(dist < .01))
    assert result['trimmed_rmse_m'] == pytest.approx(np.sqrt(np.mean(np.sort(dist)[:int(.8*len(dist))]**2)))


def test_sparse_coarse_voxels_reuse_fine_cloud():
    points = np.array([[x*.0031, y*.0031, z*.0031] for x in range(6)
                       for y in range(6) for z in range(2)])
    model = np.repeat(points, 3, axis=0)
    normals = np.tile([0., 0., 1.], (len(model), 1))
    kwargs = dict(initial_angles=12, yaw_span_deg=180.)
    baseline = fit_icp(points, model, normals, np.array([0., 0., 1.]), **kwargs)
    coarse = fit_icp(points, model, normals, np.array([0., 0., 1.]),
                     coarse_voxel_size_m=.006, **kwargs)
    assert coarse['coarse_fallback'] and coarse['coarse_voxel_points'] < 40
    assert coarse['coarse_fit_points'] == coarse['fit_points'] >= 40
    np.testing.assert_array_equal(coarse['T_reference_model'], baseline['T_reference_model'])


@pytest.mark.parametrize('kwargs', [{'coarse_voxel_size_m': -.006},
                                  {'coarse_voxel_size_m': .002},
                                  {'coarse_voxel_size_m': float('nan')},
                                  {'coarse_voxel_size_m': .006, 'coarse_point_limit': 400}])
def test_invalid_coarse_voxel_configuration(kwargs):
    with pytest.raises(ValueError, match='invalid ICP'):
        fit_icp(np.empty((0, 3)), np.empty((0, 3)), np.empty((0, 3)), np.zeros(3), **kwargs)


def test_instance_mask_excludes_foreground_gripper_and_background():
    from deux_task_perception.perception_core import masked_depth
    depth=np.full((40,50),2.,dtype=np.float32)
    depth[8:30,10:35]=1.
    depth[0:7,:]=.4  # A closer gripper outside the visible bag polygon.
    masked=masked_depth(depth,[10.,8.,34.,8.,34.,29.,10.,29.])
    assert np.isnan(masked[:8]).all() and np.isnan(masked[:,35:]).all()
    assert np.all(masked[np.isfinite(masked)]==1.)
    assert np.isfinite(masked[15,20])
    with pytest.raises(ValueError):masked_depth(depth,[0.,0.,float('nan'),2.,4.,4.])
