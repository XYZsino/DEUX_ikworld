"""Detection/ICP only; observation and camera drivers are launched separately."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('deux_task_perception'), 'config', 'perception.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        *[Node(package='deux_task_perception', executable=name, name=name, output='screen',
               parameters=[LaunchConfiguration('config'),
                           {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}])
          for name in ('detection_node', 'icp_node')]])
