"""Loopback web control for an existing ROS graph; use SSH port forwarding remotely."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults={'deployment':'real','host':'127.0.0.1','port':'8080','action_name':'/task/run','status_topic':'/task/status','ready_action_name':'/task/move_to_ready',
              'observation_topic':'/task/observation','detections_topic':'/task/detections','camera_base_frame':'base_link'}
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
        Node(package='deux_task_gui',executable='task_gui',output='screen',parameters=[{
            k:ParameterValue(LaunchConfiguration(k),value_type=int) if k=='port' else LaunchConfiguration(k)
            for k in defaults}])])
