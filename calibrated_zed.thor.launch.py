"""Camera and calibrated TF only; no robot command nodes."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    root=Path(__file__).resolve().parent
    return LaunchDescription([
        ExecuteProcess(cmd=[str(root/".venv/bin/python"),str(root/"tools/thor_camera_tf.py")],output="screen"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory("zed2i_camera"))/"launch/zed2i.launch.py")),launch_arguments={"serial_number":"33283654","depth_mode":"ULTRA","camera_fps":"15"}.items()),
    ])
