"""Overlay only. Drivers/teleop are never launched; dispatch disabled by default."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("scene_config", default_value=""),
        DeclareLaunchArgument("hand_mapping", default_value=""),
        DeclareLaunchArgument("hardware_enabled", default_value="false"),
        DeclareLaunchArgument("dry_run", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("barcode_topic", default_value="/barcode/raw"),
    ]
    common = {
        "use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool
        )
    }

    def node(package, name, extra=None):
        parameters = dict(common)
        if package in ("deux_task_runtime", "deux_task_adapters"):
            parameters["config_file"] = LaunchConfiguration("scene_config")
        return Node(
            package=package,
            executable=name,
            name=name,
            output="screen",
            parameters=[parameters, extra or {}],
        )

    return LaunchDescription(
        args
        + [
            node("deux_task_runtime", "motion_planner_node"),
            node("deux_task_runtime", "task_node"),
            node(
                "deux_task_runtime",
                "execution_node",
                {
                    "dry_run": ParameterValue(
                        LaunchConfiguration("dry_run"), value_type=bool
                    )
                },
            ),
            node(
                "deux_task_adapters",
                "arm_bridge_node",
                {
                    "hardware_enabled": ParameterValue(
                        LaunchConfiguration("hardware_enabled"), value_type=bool
                    )
                },
            ),
            node(
                "deux_task_adapters",
                "hand_bridge_node",
                {
                    "hardware_enabled": ParameterValue(
                        LaunchConfiguration("hardware_enabled"), value_type=bool
                    ),
                    "mapping_file": LaunchConfiguration("hand_mapping"),
                },
            ),
            node(
                "deux_task_perception",
                "barcode_bridge_node",
                {"input_topic": LaunchConfiguration("barcode_topic")},
            ),
        ]
    )
