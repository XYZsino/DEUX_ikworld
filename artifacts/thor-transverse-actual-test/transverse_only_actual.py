"""Operator-requested one-shot transverse-only trial; all production guards kept."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path('/home/xyz_thor/DEUX_ikworld/tools')))
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from classified_grasp_trial import ClassifiedTrial
from classified_grid_runtime import transverse_yaw


class TransverseOnlyActual(ClassifiedTrial):
    def tracked_observation(self):
        observation=super().tracked_observation()
        pose=observation[1]
        yaw=float(np.degrees(np.arctan2(pose[1,0],pose[0,0])))
        if not transverse_yaw(yaw):
            raise ValueError(f'transverse-only actual test: observed yaw {yaw:.1f}deg is outside +/-90deg +/-30deg')
        return observation

    def phase_details(self):
        details=super().phase_details();details.update(test_mode='transverse-only one-shot actual test')
        return details


if __name__=='__main__':
    rclpy.init();node=TransverseOnlyActual()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
