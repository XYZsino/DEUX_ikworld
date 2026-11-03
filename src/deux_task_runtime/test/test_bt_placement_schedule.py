import numpy as np
from rclpy.action import ActionClient
from deux_task_msgs.action import PlanMotion,ExecuteMotion
from deux_task_core.synthetic import fixture
from deux_task_core.planning import scene_version
from deux_task_core.placement import stages
from deux_task_runtime.motion_planner_node import MotionPlannerNode
from deux_task_runtime.execution_node import ExecutionNode
from deux_task_runtime.common import seconds
from test_nodes import Harness,params

def test_static_bt_plan_transmits_full_placement_schedule(tmp_path):
 c=fixture(tmp_path);c.update(perception_policy='head_rgbd',scan_right_joints=[0.]*7,observe_right_joints=[0.]*7,stationary_source_confirmed=True,grasp_pose_max_age=2.,hand_open_duration_s=.5)
 release=np.array(c['destination_pose']);hover=release.copy();hover[2,3]+=.1
 def q(x):return [x,0,0,0,0,0,0]
 c['placement']=dict(enabled=True,calibrated=False,clear_left_joints=q(0),hover_right_pose=hover.tolist(),release_right_pose=release.tolist(),position_tolerance_m=.02,rotation_tolerance_rad=.1,left_tolerance_rad=.06,hand_tolerance_rad=.1,fixed_joint_motion=True,continuous_motion=True,joint_motion_scale=.5,prevalidate_descent=True,fixed_joint_waypoints={'PlaceApproach':[q(.05),q(.1)],'Place':[q(.15),q(.2)],'PlaceRetreat':[q(.15),q(.1)],'Retreat':[q(0)]})
 h=Harness(c);h.q[7:14]=c['hand_closed'][:7];h.q[21:]=c['hand_closed'][7:]
 planner=h.add(MotionPlannerNode(parameter_overrides=params(config_file=str(h.config_path))))
 executor=h.add(ExecutionNode(parameter_overrides=params(config_file=str(h.config_path))))
 client=ActionClient(h.source,PlanMotion,'/task/plan_motion');execute=ActionClient(h.source,ExecuteMotion,'/task/execute_motion')
 try:
  h.spin(lambda:planner.observation is not None and executor.observation is not None and client.server_is_ready() and execute.server_is_ready())
  assert stages(c)==('ClearReader','PlaceApproach')
  handle=h.future(client.send_goal_async(PlanMotion.Goal(stage='PlaceApproach',target_id='',scene_version=scene_version(h.config),observation=h.observation)))
  assert handle.accepted
  result=h.future(handle.get_result_async(),20).result
  assert result.success,result.reason
  p=result.plan;assert not p.follows_conveyor
  assert len(p.phase_end_times)==3
  a,b,end=p.phase_end_times
  assert a>0 and abs(b-a-.5)<1e-9 and abs(end-seconds(p.trajectory.points[-1].time_from_start))<1e-6
  points=np.array([v.positions for v in p.trajectory.points]);t=np.array([seconds(v.time_from_start) for v in p.trajectory.points])
  hold=points[(t>=a-1e-8)&(t<=b+1e-8),14:21]
  np.testing.assert_allclose(hold,np.tile(q(.2),(len(hold),1)),atol=1e-9)
  np.testing.assert_allclose(points[-1,14:21],q(0),atol=1e-9)
  accepted=h.future(execute.send_goal_async(ExecuteMotion.Goal(plan=p)));assert accepted.accepted
  done=h.future(accepted.get_result_async()).result;assert done.success,done.reason
 finally:h.close()
