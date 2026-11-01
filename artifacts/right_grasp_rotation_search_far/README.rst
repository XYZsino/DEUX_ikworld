Rotation exploration (offline only)
==================================

Tested 5565 combinations: 105 contact cells x 53 rotations.
XY grid: X 0.54..0.72m, Y -0.18..0.24m, 0.03m spacing; base_link Z=0.97m.
Rotations: reference plus 26 signed axis/face/body-diagonal directions at 15 and 29 degrees.
R_target = Exp(rotvec_base) @ R_reference. This is not independent Euler +/-30deg.
Target fixed through descent/lift/braking (<=1deg sampled error); <=30deg original cone.
Ready pose identical to installation.yaml. Contact 3s; close 2.5..3s; lift 3..4.5s; stop 5s.
Speed fixed at base -Y 0.09m/s; upper clearance 8cm, lift 10cm.

Accepted 2 paths across 2 cells; 0 additional cells.
Previous relaxed-cone paths are preserved and are not replaced by fixed rotations.
manifest.json contains every attempted combination, failure stage, hashes and assumptions.
summary.json contains per-rotation counts and independent FK/derivative verification.

Limitations
-----------

Existing grasp spheres and observed conveyor envelopes only; full body/self/held-object collision unverified.
Approach is a joint-space cubic to moving descent; no Cartesian straight-line guarantee during approach.
Position/orientation and collision sampled every 10ms; joint polynomial extrema checked analytically.
Cone applies from descent start through lifted stop; ready orientation may lie outside cone.
Fixed -Y speed and fixed object orientation; velocity changes require another validated library.
Hand closure is a command schedule, not physical contact verification; terminal stop is included.
Finite rotation and XY search, not global reachability or optimality proof.
Fixed rotations tracked within 1deg; original 30deg cone checked independently.
Static left arm is installation home, not fresh hardware feedback.
Separate offline artifact, not compatible with live GridLibrary dispatch schema.

No fresh hardware-left-arm audit, torque/payload check, hand closure confirmation or physical grasp test.
Do not deploy this research schema to GridLibrary without explicit conversion and runtime validation.
Failures indicate this deterministic solver/path search failed, not mathematical infeasibility.

Reproduce
---------

  /usr/bin/python3 tools/explore_grasp_rotations.py
  /usr/bin/python3 tools/summarize_grasp_rotations.py
