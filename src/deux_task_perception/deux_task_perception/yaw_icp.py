"""Yaw-only ICP adapted from the local zed_ros2_wrapper_2d/fast_fit.py.

Robust bidirectional point-to-point fit; translation XYZ, rotation about
reference Z only. Metrics describe fit residuals, not pose accuracy.
"""
import time, numpy as np
from scipy.spatial import cKDTree
from .perception_core import downsample, rot, solve_yaw
CONFIG = dict(initial_angles=24, coarse_iterations=5, refine_candidates=2, refine_iterations=20, plateau_patience=3, rmse_absolute_tolerance_m=3e-05, rmse_relative_tolerance=0.002)

def fit_icp(points, model, normals, camera):
    start = time.perf_counter()
    q = downsample(points)
    m = model[::3]
    norm = normals[::3]
    if len(q) < 40:
        return dict(status='insufficient_depth', points=len(points), fit_points=len(q), seconds=time.perf_counter() - start)
    tree = cKDTree(m)
    target = cKDTree(q)
    center = (np.quantile(q, 0.1, axis=0) + np.quantile(q, 0.9, axis=0)) / 2
    mc = (m.min(0) + m.max(0)) / 2

    def score(theta, t):
        R = rot(theta)
        dist = tree.query((q - t) @ R)[0]
        rmse = float(np.sqrt(np.mean(np.sort(dist)[:max(1, int(len(dist) * 0.8))] ** 2)))
        cov = float(np.mean(dist < 0.01))
        transformed = m @ R.T + t
        visible = np.sum(norm @ R.T * (camera - transformed), axis=1) > 0
        sd = target.query(transformed[visible])[0]
        mr = float(np.sqrt(np.mean(np.sort(sd)[:max(1, int(len(sd) * 0.8))] ** 2))) if len(sd) else 0.1
        mv = float(np.mean(sd < 0.01)) if len(sd) else 0.0
        return dict(theta=float(theta), translation=t.tolist(), score=rmse + 0.35 * mr + 0.015 * (1 - cov), trimmed_rmse_m=rmse, target_coverage_10mm=cov, visible_model_rmse_m=mr, visible_model_coverage_10mm=mv)

    def run(theta, t, t0, limit, offset, early):
        previous = None
        stable = 0
        reason = 'iteration_limit'
        iterations = 0
        for step in range(limit):
            R = rot(theta)
            dist, idx = tree.query((q - t) @ R)
            err = float(np.sqrt(np.mean(np.sort(dist)[:max(1, int(len(dist) * 0.8))] ** 2)))
            if early and step >= 3 and (offset + step >= 8) and (previous is not None):
                stable = stable + 1 if abs(err - previous) < max(CONFIG['rmse_absolute_tolerance_m'], previous * CONFIG['rmse_relative_tolerance']) else 0
                if stable >= CONFIG['plateau_patience']:
                    reason = 'rmse_plateau'
                    break
            previous = err
            keep = dist <= min(0.08 if offset + step < 8 else 0.05, np.quantile(dist, 0.8))
            p = m[idx[keep]]
            obs = q[keep]
            w = 1 / (1 + (dist[keep] / 0.015) ** 2)
            transformed = m @ R.T + t
            visible = np.sum(norm @ R.T * (camera - transformed), axis=1) > 0
            vid = np.flatnonzero(visible)[::3]
            sd, si = target.query(transformed[vid])
            ok = sd <= min(0.06, np.quantile(sd, 0.8)) if len(sd) else np.zeros(0, bool)
            if ok.any():
                wp = 0.3 * len(w) / ok.sum() / (1 + (sd[ok] / 0.015) ** 2)
                p = np.concatenate([p, m[vid[ok]]])
                obs = np.concatenate([obs, q[si[ok]]])
                w = np.r_[w, wp]
            if len(w) < 20:
                reason = 'insufficient_correspondences'
                break
            nxt, nt = solve_yaw(p, obs, w)
            nt = np.clip(nt, t0 - 0.15, t0 + 0.15)
            change = np.linalg.norm(nt - t) + abs(np.arctan2(np.sin(nxt - theta), np.cos(nxt - theta))) * 0.1
            theta, t = (nxt, nt)
            iterations += 1
            if early and change < 2e-05:
                reason = 'pose_converged'
                break
        return (theta, t, iterations, reason)
    candidates = []
    for i, theta0 in enumerate(np.linspace(-np.pi, np.pi, CONFIG['initial_angles'], endpoint=False)):
        t0 = center - mc @ rot(theta0).T
        theta, t, it, reason = run(theta0, t0.copy(), t0, CONFIG['coarse_iterations'], 0, False)
        c = score(theta, t)
        c.update(candidate_id=i, initial_theta=float(theta0), initial_translation=t0.tolist(), coarse_iterations=it, refine_iterations=0, iterations=it, refined=False, stop_reason=reason)
        candidates.append(c)
    order = sorted(range(len(candidates)), key=lambda i: candidates[i]['score'])[:CONFIG['refine_candidates']]
    for i in order:
        c = candidates[i]
        theta, t, it, reason = run(c['theta'], np.array(c['translation']), np.array(c['initial_translation']), CONFIG['refine_iterations'], c['coarse_iterations'], True)
        new = score(theta, t)
        if new['score'] < c['score']:
            c.update(new)
            c['pose_stage'] = 'refined'
        else:
            c['pose_stage'] = 'coarse'
        c.update(refine_iterations=it, iterations=c['coarse_iterations'] + it, refined=True, stop_reason=reason)
    candidates.sort(key=lambda c: c['score'])
    best = candidates[0]
    R = rot(best['theta'])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = best['translation']
    np.testing.assert_allclose(R[:, 2], [0, 0, 1], atol=1e-12)
    return dict(status='fitted', points=len(points), fit_points=len(q), yaw_deg=float(np.degrees(best['theta'])), T_reference_model=T.tolist(), seconds=time.perf_counter() - start, **best, multistart=candidates, algorithm='coarse_selection_early_stop', config=CONFIG, total_iterations=sum((c['iterations'] for c in candidates)), refined_candidates=len(order))
