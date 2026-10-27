from copy import deepcopy
from types import SimpleNamespace
import numpy as np
import pytest
from deux_task_core.placement import validate,check_start,enabled
from deux_task_core.planning import requires_object_pose


def configuration():
    r=np.eye(4);r[:3,3]=[.2,-.5,.7];h=r.copy();h[2,3]+=.15
    return dict(synthetic=True,perception_policy='head_rgbd',hand_open=[0.]*14,
        destination_bounds=dict(min=[.1,-.6,.4],max=[.3,-.4,.8]),placement=dict(enabled=True,calibrated=False,
            clear_left_joints=[0.]*7,hover_right_pose=h.tolist(),release_right_pose=r.tolist(),
            position_tolerance_m=.02,rotation_tolerance_rad=.1,left_tolerance_rad=.06,
            hand_tolerance_rad=.1))


def test_fixed_ee_placement_does_not_request_stale_icp():
    c=configuration();validate(c)
    assert not requires_object_pose(c,'Place')
    assert requires_object_pose(c,'Approach')
    c['placement']['enabled']=False
    assert requires_object_pose(c,'Place')  # legacy implementation retained
    c['placement']['enabled']=True;c['synthetic']=False
    with pytest.raises(ValueError,match='calibrated'):validate(c)
    c['placement']['calibrated']=True;validate(c)


@pytest.mark.parametrize('case',['tilted_hover','horizontal_descent','outside_bin','nan','bad_tolerance'])
def test_invalid_installation_rejected(case):
    c=configuration();p=c['placement']
    if case=='tilted_hover':p['hover_right_pose'][0][0]=.8
    if case=='horizontal_descent':p['hover_right_pose'][0][3]+=.1
    if case=='outside_bin':p['release_right_pose'][0][3]=p['hover_right_pose'][0][3]=1.
    if case=='nan':p['position_tolerance_m']=float('nan')
    if case=='bad_tolerance':p['position_tolerance_m']=.5
    with pytest.raises(ValueError):validate(c)


def test_release_requires_reader_clearance_and_release_waypoint():
    c=configuration();q=np.zeros(28);pose=np.array(c['placement']['release_right_pose'])
    robot=SimpleNamespace(ee={'right':'right'},fk=lambda _: {'right':pose})
    check_start(robot,c,'Release',q)
    q[0]=.2
    with pytest.raises(ValueError,match='cleared left'):check_start(robot,c,'Release',q)
    q[0]=0;pose[2,3]+=.1
    with pytest.raises(ValueError,match='waypoint'):check_start(robot,c,'Release',q)
    pose[2,3]-=.1;q[21]=.3
    with pytest.raises(ValueError,match='open right'):check_start(robot,c,'PlaceRetreat',q)
    q[21]=0;check_start(robot,c,'PlaceRetreat',q)


def test_clear_reader_holds_loaded_right_arm_and_release_does_not_move_arms(tmp_path):
    from deux_task_core.synthetic import fixture
    from deux_task_core.planning import Planner
    c=fixture(tmp_path);c.update(perception_policy='head_rgbd',scan_right_joints=[0.]*7,observe_right_joints=[0.]*7,
                               stationary_source_confirmed=True,grasp_pose_max_age=2.)
    p=configuration()['placement'];r=np.array(c['destination_pose']);h=r.copy();h[2,3]+=.1
    p.update(release_right_pose=r.tolist(),hover_right_pose=h.tolist(),clear_left_joints=[.1,0,0,0,0,0,0]);c['placement']=p
    planner=Planner(c);q=np.zeros(28);q[7:14]=c['hand_closed'][:7];q[21:28]=c['hand_closed'][7:]
    clear=planner.plan('ClearReader',q,None)['path']
    np.testing.assert_allclose(clear[:,14:21],0.,atol=1e-10)
    np.testing.assert_allclose(clear[:,21:28],.03,atol=1e-10)
    release=planner.plan('Release',clear[-1],None)['path']
    np.testing.assert_allclose(release[:,14:21],0.,atol=1e-10)
    np.testing.assert_allclose(release[-1,21:28],0.,atol=1e-10)
    np.testing.assert_allclose(release[:,7:14],.03,atol=1e-10)


@pytest.mark.parametrize('grip',['open','closed','mixed'])
def test_ready_returns_arms_without_opening_or_closing_hands(tmp_path,grip):
    from deux_task_core.synthetic import fixture
    from deux_task_core.planning import Planner
    c=fixture(tmp_path);q=np.zeros(28);q[0]=.1;q[14]=-.1
    if grip in ('closed','mixed'):q[21:28]=c['hand_closed'][7:]
    if grip=='closed':q[7:14]=c['hand_closed'][:7]
    result=Planner(c).plan('ReadyPose',q,None)
    assert not requires_object_pose(c,'ReadyPose')
    np.testing.assert_allclose(result['path'][-1,list(range(7))+list(range(14,21))],0.,atol=1e-10)
    for indices in (list(range(7,14)),list(range(21,28))):
        np.testing.assert_allclose(result['path'][:,indices],np.tile(q[indices],(len(result['path']),1)),atol=1e-10)
