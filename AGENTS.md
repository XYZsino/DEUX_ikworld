# Working context

- Start by reading `docs/HANDOFF.rst` and `README.rst`, then inspect the current code.
- This workspace contains the new ROS 2 nodes for transferring the DEUX MuJoCo task to real hardware. Add new nodes here, as explicitly requested by the user.
- Do not modify existing code in `/home/xyz-ai/git/DEUX`; use its interfaces from additive packages.
- Use ROS 2 Jazzy. `tools/validate_jazzy.sh` performs a disposable container build and synthetic ROS tests without connecting hardware.
- The external ZED camera is separate from the old head camera. Do not invent extrinsics or silently use the old URDF camera pose.
- Distinguish implemented functionality, proposed design, synthetic test evidence, and actual hardware verification.
- For any local HTML report, show the complete copyable `file:///` URL. Open it in the user's browser when available and report the outcome honestly.
