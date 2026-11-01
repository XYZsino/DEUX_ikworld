"""ROS entry for ordinary bags and axis-preserving +/-90deg bags. Preview default."""
import rclpy
from rclpy.executors import ExternalShutdownException
from grid_grasp_trial import GridTrial
from classified_grid_runtime import ClassifiedGridLibrary


class ClassifiedTrial(GridTrial):
    library_class=ClassifiedGridLibrary

    def phase_details(self):
        return dict(workflow='orientation-classified stored grasp',
                    selection_policy='abs(yaw) 60..120deg: transverse axis <=3deg, no normal fallback; otherwise normal XY-first soft yaw',
                    rotation_preference='transverse: axis alignment required; normal: soft yaw preference',
                    normal_contact_cells=len(self.library.normal_options),
                    transverse_contact_cells=len(self.library.transverse_options),
                    front_back_reversal_allowed=True)


def main():
    rclpy.init();node=ClassifiedTrial()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
