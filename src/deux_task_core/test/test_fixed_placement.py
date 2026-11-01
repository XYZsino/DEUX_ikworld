import numpy as np
import pytest
from deux_task_core.synthetic import fixture
from deux_task_core.planning import Planner
from deux_task_core.placement import check_start,validate,stages


def configured(tmp_path):
    c=fixture(tmp_path)
    c.update(perception_policy='head_rgbd',scan_right_joints=[0.]*7,observe_right_joints=[0.]*7,
             stationary_source_confirmed=True,grasp_pose_max_age=2.)
    release=np.array(c['destination_pose']);hover=release.copy();hover[2,3]+=.1
    def q(x):return [x,0,0,0,0,0,0]
    c['placement']=dict(enabled=True,calibrated=False,clear_left_joints=q(0),
        hover_right_pose=hover.tolist(),release_right_pose=release.tolist(),
        position_tolerance_m=.02,rotation_tolerance_rad=.1,left_tolerance_rad=.06,hand_tolerance_rad=.1,
        fixed_joint_motion=True,joint_motion_scale=.5,prevalidate_descent=True,
        fixed_joint_waypoints={'PlaceApproach':[q(.05),q(.1)],'Place':[q(.15),q(.2)],
                              'PlaceRetreat':[q(.15),q(.1)],'Retreat':[q(0)]})
    return c


def test_complete_fixed_flow_never_solves_ik_and_preserves_grip(tmp_path):
    c=configured(tmp_path);planner=Planner(c)
    def forbidden(*a,**kw):raise AssertionError('runtime IK called')
    planner.robot.ik=forbidden
    q=np.zeros(28);q[7:14]=c['hand_closed'][:7];q[21:28]=c['hand_closed'][7:]
    for stage in stages(c):
        result=planner.plan(stage,q,None);path=result['path'];q=path[-1]
        if stage in c['placement']['fixed_joint_waypoints']:
            np.testing.assert_allclose(q[14:21],c['placement']['fixed_joint_waypoints'][stage][-1],atol=1e-10)
        np.testing.assert_allclose(q[7:14],c['hand_closed'][:7])
        if stage in ('Release','PlaceRetreat','Retreat'):
            np.testing.assert_allclose(q[21:28],c['hand_open'][7:],atol=1e-10)
        else:np.testing.assert_allclose(q[21:28],c['hand_closed'][7:])


def test_fixed_release_still_requires_arrival_and_limits(tmp_path):
    c=configured(tmp_path);planner=Planner(c);q=np.zeros(28)
    q[14]=.2;check_start(planner.robot,c,'Release',q)
    q[14]=1.
    with pytest.raises(ValueError):check_start(planner.robot,c,'Release',q)
    c['placement']['fixed_joint_waypoints']['PlaceApproach'][-1][0]=100.
    with pytest.raises(ValueError,match='joint limit'):planner.plan('PlaceApproach',np.zeros(28),None)


def test_missing_fixed_waypoint_rejected(tmp_path):
    c=configured(tmp_path);del c['placement']['fixed_joint_waypoints']['PlaceRetreat']
    with pytest.raises(ValueError,match='missing fixed'):validate(c)
