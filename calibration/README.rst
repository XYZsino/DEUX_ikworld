Black-box bottom plane: Thor, 2026-10-29
======================================
450 frames over 59.87 seconds; ZED 2i serial 33283654, ULTRA, HD720.
Optical coordinates: x right, y down, z forward; metric depth.
Black-box interior floor only; no walls or table included in manual ROI.
Temporal median/MAD filtering, confidence <=70, RANSAC and Tukey IRLS.
First 225 frames select plane-support region; last 225 independently validate.
Final support: 12207 pixels. Fit residual RMSE 1.047 mm.
Plane: -0.00092463*x -0.62312559*y -0.78212124*z +0.66623679 = 0.
Offset is camera-to-plane perpendicular distance, NOT floor height.
Train/test normals differ 0.7815 deg; offsets differ 6.807 mm.
Spatial-half normals differ from final normal by up to 1.2778 deg.
Held-out residual p95: 4.010 mm. Precision gate remains FAILED.
Result is provisional, not calibrated robot-base TF; do not interpret fit
residual as absolute accuracy. Stereo bias/reflection/shape may remain.
No active TF/previous reference was changed. Floor-to-shoulder measurement
1.340m is recorded; base origin floor height and box elevation remain unknown.
Raw evidence: ../.setup/black-box-plane-450/.
Capture: ../tools/capture_black_box_450.py
Fit: ../tools/fit_black_box_450.py
Independent support validation: ../tools/validate_black_box_core.py

Accepted room-floor plane (2026-10-29)
====================================
Use floor_plane.thor.json as the accepted surface measurement.
450 frames, camera serial 33283654; actual floor, cables/legs excluded.
Final support count: 46448 pixels (see JSON).
Normal in optical coordinates: [0.000486473,-0.727623868,-0.685976144].
Plane n dot p + 1.351467873 = 0 (metres).
Camera perpendicular height above observed floor: 1.35147 m.
Independent first/last 225-frame normals differ 0.04443 deg, offsets 0.139 mm.
Temporal max normal deviation 0.06360 deg; spatial-half max 0.15714 deg.
Held-out p95 residual 2.512 mm, fit inlier RMSE 1.187 mm.
These are repeatability/fit metrics, not certified absolute accuracy.
Plane fit accepted. Robot base calibration is NOT completed or published.
Plane normal constrains two rotations; yaw must come from URDF or another
heading measurement. Do not use the arbitrary floor-frame yaw as robot yaw.
Floor-frame XY origin is the camera perpendicular projection, not base_link.
User shoulder height 1.340 m is recorded; base_link floor offset is unconfirmed.
Evidence: ../.setup/floor-plane-450/plane-core.json and plane-core-overlay.jpg.
Reproduce: tools/capture_floor_plane.py then tools/validate_floor_plane.py
using .venv-zed and .venv respectively. Camera capture has been closed.
