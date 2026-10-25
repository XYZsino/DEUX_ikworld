"""Native RGB-D shared detector plus image-free control state; drivers stay external."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        *[Node(package='deux_task_perception', executable=name, output='screen',
               parameters=[LaunchConfiguration('config'), {'use_sim_time': ParameterValue(
                   LaunchConfiguration('use_sim_time'), value_type=bool)}])
          for name in ('perception_node', 'state_observation_node')]])
