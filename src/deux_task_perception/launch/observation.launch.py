"""Launch only the observation node; do not launch cameras or robot drivers."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    default = os.path.join(get_package_share_directory('deux_task_perception'),
                           'config', 'observation.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='deux_task_perception', executable='observation_node',
             name='observation_node', output='screen',
             parameters=[LaunchConfiguration('config'),
                         {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'),
                                                        value_type=bool)}]),
    ])
