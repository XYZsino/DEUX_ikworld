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
