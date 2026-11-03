"""Reproduce the offline attention -> retract/lift -> ready candidate; no ROS."""
import hashlib
import json
import numpy as np
from scipy.optimize import least_squares
from build_right_grasp_grid import Builder, ROOT
from lookup_conveyor import validate_conveyor_path


def build():
    b = Builder(ROOT/'src/deux_task_bringup/config/installation.yaml')
    start = b.ready.copy(); start[14:21] = 0
    seed = np.radians([-40, 10, 0, -75, -70, 0, 0])
    lower = b.lo.copy(); lower[3] = np.radians(-115)  # 5 deg elbow limit reserve
    waypoints = [np.zeros(7)]
    for target in ([.237, -.103, .9], [.30, -.12, 1.05]):
        def residual(arm):
            q = start.copy(); q[14:21] = arm
            hand = b.robot.ee_fk('right', q)[:3, 3]
            elbow = b.robot.fk(q)['right_arm_4'][:3, 3]
            return np.r_[10*(hand-target), 30*max(0, elbow[0]-.045), .03*(arm-seed)]
        solved = least_squares(residual, seed, bounds=(lower, b.hi), max_nfev=120)
        if not solved.success or np.linalg.norm(residual(solved.x)[:3]) > .01:
            raise ValueError('compact ready waypoint optimization failed')
        waypoints.append(solved.x); seed = solved.x
    waypoints.append(b.ready[14:21])
    path = np.tile(start, (len(waypoints), 1)); path[:, 14:21] = waypoints
    margin = validate_conveyor_path(b.robot, path)
    data = dict(schema='deux.attention-ready.v1',
                urdf_sha256=hashlib.sha256(open(b.config['robot']['urdf_path'], 'rb').read()).hexdigest(),
                start_tolerance_rad=.30, right_waypoints=np.asarray(waypoints).tolist(),
                minimum_conveyor_clearance_m=margin,
                description='Right shoulder retracts while elbow folds; lift close to body, then extend to unchanged ready. Offline model only.')
    destination = ROOT/'calibration/attention_ready_route.json'
    destination.write_text(json.dumps(data, indent=2)+'\n')
    print(destination)


if __name__ == '__main__':
    build()
