"""Lookup full BT. Reuses running camera/perception/drivers; preview by default."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    root=Path(__file__).resolve().parent
    enabled=LaunchConfiguration('enable_execution').perform(context).lower()=='true'
    repeat=LaunchConfiguration('repeat_execution').perform(context).lower()=='true'
    scan_speed=float(LaunchConfiguration('scan_speed_scale').perform(context))
    parallel=LaunchConfiguration('parallel_reader_retreat').perform(context).lower()=='true'
    reading_pose=LaunchConfiguration('dataset_reading_pose_file').perform(context)
    right_ready=float(LaunchConfiguration('right_hand_ready_fraction').perform(context))
    reader_yaw=float(LaunchConfiguration('barcode_reader_extra_yaw_deg').perform(context))
    config=str(root/'src/deux_task_bringup/config/installation.yaml')
    common={'config_file':config}
    # Refuse existing task owners: launching a second coordinator/executor can
    # otherwise command the same robot from independent action servers.
    import rclpy
    from rclpy.node import Node as RosNode
    import time
    import yaml
    rclpy.init();probe=RosNode('lookup_bt_launch_preflight')
    try:
        until=time.monotonic()+2.
        while time.monotonic()<until:rclpy.spin_once(probe,timeout_sec=.1)
        names={name for name,_ in probe.get_node_names_and_namespaces()}
        conflicts=names&{'task_node','execution_node','head_task_node','tracking_icp_monitor','lookup_bt_server','lookup_perception_bridge','grid_grasp_trial','motion_planner_node'}
        if conflicts:raise RuntimeError('Stop idle conflicting nodes first: '+', '.join(sorted(conflicts)))
    finally:probe.destroy_node();rclpy.shutdown()
    nodes=[]
    if 'state_observation_node' not in names:
        scene=yaml.safe_load(Path(config).read_text())
        nodes.append(Node(package='deux_task_perception',executable='state_observation_node',
            parameters=[{'scene_config':config,'left_ee_frame':'left_grasp','right_ee_frame':'right_grasp',
                         'publish_rate_hz':50.,'max_joint_age_s':.2,'max_object_age_s':float(scene['max_object_age'])}],output='screen'))
    for script in ('lookup_bt_server','lookup_bt_node','lookup_perception_bridge'):
        args=[str(root/'.venv/bin/python'),str(root/'tools'/f'{script}.py')]
        if script!='lookup_perception_bridge':args+=['--ros-args','-p',f'config_file:={config}','-p',f'enable_execution:={str(enabled).lower()}']
        if script=='lookup_bt_node':args+=['-p',f'dataset_reading_pose_file:={reading_pose}','-p',f'repeat_execution:={str(repeat).lower()}','-p',f'scan_speed_scale:={scan_speed}','-p',f'parallel_reader_retreat:={str(parallel).lower()}']
        if script=='lookup_bt_node':
            for key in ('scan_bbox_wait_s','scan_bag_min_area_px2','scan_barcode_min_area_px2'):
                args+=['-p',f'{key}:={float(LaunchConfiguration(key).perform(context))}']
        if script=='lookup_bt_server':
            for key,value in {'right_hand_ready_fraction':right_ready,'dataset_reading_pose_file':reading_pose,'grid_library_directory':root/'artifacts/right_grasp_grid_classified',
                              'grid_output_directory':root/'.setup/lookup-bt',
                              'stationary_left_file':root/'.setup/lookup-bt/home-left-conveyor.json',
                              'use_observed_yaw':'true'}.items():args+=['-p',f'{key}:={value}']
        nodes.append(ExecuteProcess(cmd=args,cwd=str(root),output='screen'))
    for exe in ('motion_planner_node','execution_node'):
        params=dict(common,dataset_reading_pose_file=reading_pose)
        if exe=='motion_planner_node':params['barcode_reader_extra_yaw_deg']=reader_yaw
        if exe=='execution_node':params.update(hardware_enabled=enabled,dry_run=not enabled,parallel_reader_retreat=parallel,
            mapping_file=str(root/'src/deux_task_bringup/config/hand_mapping.yaml'))
        nodes.append(Node(package='deux_task_runtime',executable=exe,name=exe,parameters=[params],output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('scan_bbox_wait_s',default_value='1.0'),DeclareLaunchArgument('scan_bag_min_area_px2',default_value='1000.0'),DeclareLaunchArgument('scan_barcode_min_area_px2',default_value='300.0'),DeclareLaunchArgument('enable_execution',default_value='false'),DeclareLaunchArgument('repeat_execution',default_value='false'),DeclareLaunchArgument('scan_speed_scale',default_value='0.5'),DeclareLaunchArgument('parallel_reader_retreat',default_value='true'),DeclareLaunchArgument('barcode_reader_extra_yaw_deg',default_value='-15.0'),DeclareLaunchArgument('right_hand_ready_fraction',default_value='0.3'),DeclareLaunchArgument('dataset_reading_pose_file',default_value=str(Path(__file__).resolve().parent/'artifacts/dataset-barcode-pose/reading-pose.json')),OpaqueFunction(function=start)])
