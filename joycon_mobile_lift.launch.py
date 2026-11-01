"""Independent of deux_leader. Uses repo venv for command processing."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root=Path(__file__).resolve().parent
    actions=[DeclareLaunchArgument('check_only',default_value='false')]
    for side,label in [('left','L'),('right','R')]:
        actions.append(DeclareLaunchArgument(side+'_device',default_value=f'Nintendo Switch Joy-Con ({label})'))
        actions.append(Node(package='joy',executable='joy_node',name='ikworld_joy_'+side,
            parameters=[{'device_name':LaunchConfiguration(side+'_device'),'autorepeat_rate':20.,'deadzone':0.}],
            remappings=[('joy','/ikworld/joy/'+side),('joy/set_feedback','/ikworld/joy/'+side+'/set_feedback')],
            additional_env={'SDL_JOYSTICK_HIDAPI_COMBINE_JOY_CONS':'0','SDL_JOYSTICK_HIDAPI_VERTICAL_JOY_CONS':'1'},
            output='screen'))
    actions.append(Node(executable=str(root/'.venv/bin/python'),
        arguments=[str(root/'tools/joycon_mobile_lift_node.py')],
        parameters=[{'check_only':ParameterValue(LaunchConfiguration('check_only'),value_type=bool)}],output='screen'))
    return LaunchDescription(actions)
