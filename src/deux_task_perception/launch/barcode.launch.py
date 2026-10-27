"""Standalone Newland reader; configure enabled/port in the parameter file."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('deux_task_perception'), 'config', 'barcode.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default),
        Node(package='deux_task_perception', executable='barcode_node', name='barcode_node',
             output='screen', parameters=[LaunchConfiguration('config')]),
    ])
