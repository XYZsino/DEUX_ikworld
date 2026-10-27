"""Shared rendered RGB-D ROS stack for recorded and interactive simulations."""
from pathlib import Path
from .validate import params,StateObservationNode,MotionPlannerNode,ExecutionNode,BarcodeNode
from .node import MujocoNode
from deux_task_perception.tracked_perception_node import TrackedPerceptionNode
from deux_task_runtime.head_task_node import HeadTaskNode


def build_head_stack(sim,config,output,weights,*,device='cuda:0',record=False):
    from deux_conveyor import BREAD_ASSET,BREAD_MODEL
    output=Path(output)
    common=dict(config_file=str(output/'installation.yaml'),use_sim_time=True)
    driver=MujocoNode(sim,config,output,rgbd=True,record=record)
    dimensions=dict(width=driver.source.width,height=driver.source.height)
    perception=TrackedPerceptionNode(parameter_overrides=params(**dimensions,**common,
        weights=str(Path(weights).resolve()),device=device,model_path=str(BREAD_ASSET/BREAD_MODEL['mesh']),
        icp_track_max_age_s=config['conveyor_tracking']['occlusion_prediction_max_age'],
        filter_motion=True,filter_velocity_prior_enabled=bool(config.get('moving_source',False)),
        filter_initial_velocity=config['conveyor_tracking']['nominal_velocity'],position_std_m=.008,yaw_std_rad=.08,model_origin='bounds_center',reference_frame='source_support',base_frame='base_link',
        target_classes=['bread_bag'],max_distance_m=3.,roi_only_depth=True,require_instance_mask=True,resting_on_reference_plane=True,
        icp_initial_angles=12,icp_coarse_point_limit=400))
    nodes=[driver,perception,
        StateObservationNode(parameter_overrides=params(use_sim_time=True,left_ee_frame='left_grasp',right_ee_frame='right_grasp')),
        MotionPlannerNode(parameter_overrides=params(**common)),ExecutionNode(parameter_overrides=params(**common,dry_run=False,hardware_enabled=True,mapping_file=str(output/'hand_mapping.yaml'))),
        BarcodeNode(parameter_overrides=params(use_sim_time=True,simulated=True)),HeadTaskNode(parameter_overrides=params(**common))]
    return nodes
