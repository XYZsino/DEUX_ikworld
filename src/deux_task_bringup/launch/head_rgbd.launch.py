"""Image-first PickScan overlay. Camera, TF, robot and reader drivers run separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    required=['scene_config','hand_mapping','observation_config','weights','model_path','reference_frame']
    arguments=[DeclareLaunchArgument(name) for name in required]
    arguments += [DeclareLaunchArgument(name,default_value=value) for name,value in
                  [('use_sim_time','false'),('hardware_enabled','false'),('dry_run','true'),
                   ('device','cpu'),('barcode_enabled','false'),('barcode_port',''),('barcode_serial_number',''),('resting_on_reference_plane','false')]]
    arguments.append(DeclareLaunchArgument('planner_python',default_value='',
        description='Optional Python interpreter for the planner only (ROS Jazzy-compatible Python 3.12)'))
    def boolean(name):return ParameterValue(LaunchConfiguration(name),value_type=bool)
    common={'use_sim_time':boolean('use_sim_time')}
    def node(package,executable,extra=None,*,name=None,files=()):
        parameters=dict(common)
        if package == 'deux_task_runtime':
            parameters['config_file']=LaunchConfiguration('scene_config')
        parameters.update(extra or {})
        return Node(package=package,executable=executable,name=name or executable,
                    prefix=LaunchConfiguration('planner_python') if executable=='motion_planner_node' else '',
                    output='screen',parameters=[*files,parameters])
    return LaunchDescription(arguments+[
        node('deux_task_perception','observation_node',files=[LaunchConfiguration('observation_config')]),
        node('deux_task_perception','tracked_perception_node', name='perception_node',
             extra={'config_file':LaunchConfiguration('scene_config'),'weights':LaunchConfiguration('weights'),'device':LaunchConfiguration('device'),
              'model_path':LaunchConfiguration('model_path'),'model_origin':'bounds_center',
              'reference_frame':LaunchConfiguration('reference_frame'),'target_classes':['bread_bag'],
              'require_instance_mask':True,'roi_only_depth':True,'filter_motion':True,
              'resting_on_reference_plane':boolean('resting_on_reference_plane'),
              'icp_initial_angles':12,'icp_coarse_point_limit':400}),
        node('deux_task_perception','state_observation_node',
             {'left_ee_frame':'left_grasp','right_ee_frame':'right_grasp'}),
        node('deux_task_runtime','head_task_node',name='task_node'),
        node('deux_task_runtime','motion_planner_node'),
        node('deux_task_runtime','execution_node',{'dry_run':boolean('dry_run'),
             'hardware_enabled':boolean('hardware_enabled'), 'mapping_file':LaunchConfiguration('hand_mapping')}),
        node('deux_task_perception','barcode_node',{'enabled':boolean('barcode_enabled'),
             'port':LaunchConfiguration('barcode_port'),'serial_number':LaunchConfiguration('barcode_serial_number')})])
