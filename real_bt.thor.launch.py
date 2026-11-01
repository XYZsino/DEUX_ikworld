from pathlib import Path
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    r=Path(__file__).resolve().parent
    config=str(r/'src/deux_task_bringup/config/installation.yaml')
    scene=yaml.safe_load(Path(config).read_text())
    nodes=[Node(package='deux_task_perception',executable='tracked_perception_node',name='perception_node',
                parameters=[str(r/'tracked_perception.thor.yaml'),{'config_file':config}],output='screen'),
           Node(package='deux_task_perception',executable='state_observation_node',
                parameters=[{'scene_config':config,'left_ee_frame':'left_grasp','right_ee_frame':'right_grasp',
                             'publish_rate_hz':50.0,'max_joint_age_s':.2,'max_object_age_s':float(scene['max_object_age'])}],output='screen')]
    for exe in ('head_task_node','motion_planner_node','execution_node'):
        params={'config_file':config}
        if exe=='execution_node':params.update(hardware_enabled=True,dry_run=False,
            mapping_file=str(r/'src/deux_task_bringup/config/hand_mapping.yaml'))
        nodes.append(Node(package='deux_task_runtime',executable=exe,
                          name='task_node' if exe=='head_task_node' else exe,parameters=[params],output='screen'))
    return LaunchDescription(nodes)
