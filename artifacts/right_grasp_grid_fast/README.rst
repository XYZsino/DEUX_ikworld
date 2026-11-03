Moving right-arm grasp library
==============================

Offline URDF evidence only; no hardware commands or physical grasp verification.
Original slow library is preserved in artifacts/right_grasp_grid/.
Previous sequential-close fast library is backed up in
.setup/right-grasp-fast-before-parallel/.

Timing and selection
--------------------

Each 3cm cell represents the ACTUAL GRASP POINT AT CONTACT (t=3.0 s).
All paths start from installation.yaml home_joints, identical stationary ready.
Object motion: base_link (0,-0.09,0) m/s, no object rotation.
At dispatch the grasp point must be 27cm upstream (+Y) of the selected cell.
Approach allocation is saved per trajectory; continuous tracking/descent ends
at t=3.0 s. Close runs IN PARALLEL from 2.5 to 3.0 s, finishing at contact.
Lift starts at contact, ends at 4.5 s; braking stops the arm at 5.0 s.
Upper height is 8cm; lift is 10cm. Close overlaps the final 0.5s of descent
and follows the bag by 4.5cm in -Y, while Z is still decreasing.
During lift the held object follows belt velocity; after clearing it, braking
smoothly brings the arm to rest. Final XY is contact XY plus (0,-0.1575)m.

Use the existing timestamped prediction topic, convert object origin to grasp
origin, compensate acquisition age, and compute ETA to a candidate contact cell.
For -Y motion, ETA=(current_grasp_y-cell_y)/0.09. Dispatch when ETA=3.0s,
allowing actual controller start latency in the scheduled start timestamp.
Require candidate lateral X error and prediction uncertainty to fit the actual
grasp error budget; nearest-grid quantization can introduce 2.12cm error.
Do not claim the 3mm MODEL path residual is a validated live object tolerance.
If too late, no valid path, object ID/direction/speed/orientation changes,
stale observations or missed tracking, refuse/abort through the guarded runtime.
Continue monitoring latest predictions and arm/hand feedback during execution.

Playback
--------

Use only accepted manifest entries, verify source and trajectory SHA256.
JSON contains the seven named right-arm joints, times_s, positions_rad and
velocities_rad_s. Preserve cubic Hermite interpolation and all timing; positions
alone do not reproduce the validated path. Do not stop at stage boundaries:
the arm is moving at contact, throughout Close and into Lift.
Use the existing checked RightTrialHand adapter, ready fraction before start,
fraction=1 repeated every 20ms from 2.5s through 3.0s, then remain closed.
Schedule against the same actual ROS trajectory start timestamp as the arm.
Launch the hand scheduling task concurrently with the arm action; never await
arm completion or contact before starting closure. If command timing is missed,
abort rather than shifting closure after contact. Existing late-height closure
gates must not silently delay this library schedule.
tools/parallel_grid_hand.py provides close_during_descent(), a coroutine for a
future grid replay caller to run concurrently with the arm. It takes the same
scheduled start timestamp, the existing checked hand adapter, clock/pause,
goal/state guard and live-feedback qualification callbacks. It refuses a missed
start (>20ms), a command scheduling gap, clock reversal or unqualified feedback.
Its result records command timing only, with physical_closure_verified=false.
The helper is tested with a virtual clock but is not wired into an active node.
Hand driver timing and physical closure/contact require hardware verification.
Runtime feedback checks must gate lift/abort on missed closure; the nominal
offline schedule does not certify the hand will complete by 3.0s. The 0.5s
duration comes from installation.yaml; measured driver/transport delay and
physical closure duration are not newly measured or compensated by this change.
This generator does not modify, connect or start the existing ROS 2 Jazzy executor.

Validation and limitations
--------------------------

Independent URDF FK: <=3mm sampled position error, <=30deg total SO(3) angular
distance from the original reference orientation from descent start to stop.
This is an orientation cone, not three independent Euler angle allowances.
Ready/approach can initially be outside this cone.
Exact continuous joint polynomial extrema: limits and <=90% configured velocity
and acceleration. This is kinematic feasibility, not torque/payload certification.
Existing collision spheres plus measured visible conveyor capsules, sampled
every 10ms. Hidden structure, full body/self collision and held-object volume
remain unverified. Solver failures are not proofs of physical unreachability.
No online trajectory selection/dispatch integration or hardware test was performed.

Reproduce
---------

  /usr/bin/python3 tools/build_fast_grasp_grid.py
  /usr/bin/python3 tools/test_fast_grasp_grid.py
  /usr/bin/python3 tools/test_parallel_grid_hand.py
  /usr/bin/python3 tools/summarize_fast_grasp_grid.py
