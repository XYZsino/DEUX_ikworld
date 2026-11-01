Right-arm precomputed grasp library
==================================

Offline model evidence only; hardware has not been commanded or verified.

Inputs and selection
--------------------

installation.yaml and its measured-lift URDF are recorded by SHA256 in manifest.json.
Grid: base_link origin anchored, spacing 0.03 m. Actual grasp point Z=0.97 m.
Object axes assumed aligned with base_link; grasp rotation is the installation
grasp_object_from_right_ee rotation, stored explicitly in the manifest.
All 4087 points inside the configured scene XY box were tested.
202 complete paths passed. Rejections are solver/path failures, not
mathematical proofs of unreachability. Bounding boxes can contain invalid cells.

Recommended fully populated rectangle: X 0.36..0.45 m,
Y -0.27..0.15 m, 60 cells. See workspace.png.

Playback contract
-----------------

Select only an accepted file listed in manifest.json and verify its SHA256.
Each file contains named right-arm joints and sequential stages Approach,
Descend, Close, Lift. Each stage has times_s, positions_rad, velocities_rad_s.
Use cubic Hermite interpolation (positions AND velocities), with local stage
times starting at zero. Boundaries stop. Do not smooth, retime, join different
cells, or recompute IK at playback without new validation.

Start from the exact 28-joint ready snapshot in manifest.json; the left arm
is assumed stationary there. Only seven right-arm joints are exported.
First set the existing RightTrialHand ready fraction 0.5.
Approach ends at Z=1.05 m; Descend ends at Z=0.97 m. Hold the arm for the
Close stage, sending fraction=1 through the existing checked RightTrialHand
adapter at 20 ms intervals for 0.5 s. Require measured hand completion before
starting Lift to Z=1.07 m. Keep the hand closed during Lift.
Arm commands must use the existing guarded ROS 2 Jazzy action execution path,
including fresh joint state, start tolerance, tracking/error/estop checks,
controller ownership, stage completion and timeout checks. A JSON file is
not a replacement for these runtime checks. No hardware replay node was
started by these offline tools.

Each cell is an independent trial ending lifted. Return-to-ready, release,
and travel between grid cells are outside this library and are not implicit.

Validation scope
----------------

IK knot residual <=1 mm / 1 degree; interpolated vertical path <=2 mm / 2 degrees.
Cartesian knots <=5 mm apart; zero knot velocity; cubic joint velocity and
acceleration maxima analytically retimed to <=50% of URDF/config limits.
Joint positions remain inside limits by monotone cubic interpolation.
Existing grasp spheres and measured conveyor capsule screening are checked
every 10 ms. This is sampled temporal collision evidence, not continuous-time
full robot collision proof. Hidden conveyor geometry, complete body/self
collision, held-object shape and physical grasp accuracy remain unverified.
Configured grasp transform contains SIM-derived orientation and an estimated
measured-joint pinch midpoint; it is not a newly calibrated physical contact.
Close is a hardware hand command schedule, not a finger FK/contact simulation.

Reproduce
---------

  /usr/bin/python3 tools/build_right_grasp_grid.py
  /usr/bin/python3 tools/test_right_grasp_grid.py
  /usr/bin/python3 tools/summarize_right_grasp_grid.py

Python dependencies: NumPy, SciPy, PyYAML, Matplotlib. The generator explicitly
uses offline SciPy IK; installation.yaml's PyRoKi runtime choice is unchanged.
