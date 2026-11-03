"""Offline behavior regression; synthetic kinematics, no ROS or hardware clients."""
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from deux_task_core.position_first_preview import desired
from deux_task_core.tracking_policy import (
    closure_ok, follow_admitted, validate_policy, workspace_descent_ready,
)
from async_upper import compute_replacement, spline
from progressive_descent import height_state, target


@pytest.fixture
def policy():
    return validate_policy(json.loads(Path('calibration/tracking_grasp_policy.json').read_text()))


def test_outside_rotation_then_inside_xyz(policy):
    initial=np.eye(4);initial[:3,3]=[.58,-.135,1.05]
    obj=np.eye(4);obj[:3,3]=[.60,.1,.97]
    obj[:3,:3]=Rotation.from_euler('z',60,degrees=True).as_matrix()
    v=np.array([0.,-.05,0.,0.]);grasp=np.eye(4)
    assert not follow_admitted(policy,obj,v,grasp,0.)
    outside,feed=desired(initial,obj,v,grasp,.08,1.,10.,2.,0.,policy,False)
    assert outside[0,3]==pytest.approx(.60)
    np.testing.assert_allclose(outside[1:3,3],[-.135,1.05])
    assert 0<Rotation.from_matrix(outside[:3,:3]).magnitude()<np.radians(60)
    assert np.linalg.norm(feed[3:])>0
    obj[1,3]=0.
    assert follow_admitted(policy,obj,v,grasp,0.)
    inside,feed=desired(initial,obj,v,grasp,.08,1.,10.,2.,0.,policy,True)
    np.testing.assert_allclose(inside[:3,3],[.60,-.1,1.05])
    np.testing.assert_allclose(feed[:3],v[:3])


@pytest.mark.parametrize('xy,elapsed,follow,inside,roi,expected',[
    (.029,1.2,True,True,True,True),
    (.031,1.2,True,True,True,False),
    (.01,.5,True,True,True,False),
    (.01,1.2,False,True,True,False),
    (.01,1.2,True,False,True,False),
    (.01,1.2,True,True,False,False),
    (float('nan'),1.2,True,True,True,False),
])
def test_descent_requires_alignment_and_full_future_window(policy,xy,elapsed,follow,inside,roi,expected):
    assert workspace_descent_ready(policy,xy,elapsed,1.,follow,inside,roi)==expected


@pytest.mark.parametrize('value',[0.,-.01,float('nan'),.13])
def test_invalid_descent_tolerance_rejected(policy,value):
    with pytest.raises(ValueError,match='descent XY'):
        validate_policy(dict(policy,descent_xy_m=value))


class CartesianRobot:
    """Synthetic 7-DOF task coordinates; does not model real conveyor geometry."""
    limits=np.tile([-3.,3.],(28,1))
    velocity=np.full(28,2.)
    acceleration=np.full(28,3.)

    def ee_fk(self,side,q):
        pose=np.eye(4);pose[:3,3]=q[14:17]
        pose[:3,:3]=Rotation.from_rotvec(q[17:20]).as_matrix()
        return pose


def replacement_inputs(policy):
    q=np.zeros(28);q[14:17]=[.58,-.1,1.05]
    obj=np.eye(4);obj[:3,3]=[.58,-.1,.97]
    old=dict(times=[0.,1.,2.,4.],path=np.tile(q,(4,1)).tolist(),
             velocities=np.zeros((4,28)).tolist(),initial_pose=CartesianRobot().ee_fk('right',q).tolist(),
             grasp=np.eye(4).tolist(),object_at_start=obj.tolist(),velocity=[0.,-.02,0.,0.],
             start_at=100.,brake_start_s=3.,execution_end_s=8.,approach_s=.1,
             geometry={},workspace_follow=True,target_id='A')
    config=dict(tracking_grasp_policy=policy,progressive_descent=True,approach_height=.08,
                upper_replan_horizon_s=.8,tracking_lookahead_s=0.,_descent_admitted=True,
                obstacles=[],collision_margin=.02)
    return config,old,obj


@pytest.fixture
def synthetic_planner(monkeypatch):
    import async_upper as module
    monkeypatch.setattr(module,'planning_robot',lambda config:CartesianRobot())
    monkeypatch.setattr(module,'roi_contains',lambda *args:True)
    # Exercise polynomial/dynamics validation; geometry is explicitly synthetic.
    calls=[]
    def clear(robot,states,*args):
        calls.append(len(states))
        return np.full(len(states),.2)
    monkeypatch.setattr(module,'checked_clearances',clear)
    return module,calls


def test_async_descent_preserves_prefix_and_stop_tail(policy,synthetic_planner):
    module,calls=synthetic_planner
    c,old,obj=replacement_inputs(policy)
    new=compute_replacement(c,old,obj,np.array(old['velocity']),100.,.8,1)
    grid=np.linspace(0,.8,51)
    for order in (0,1):
        np.testing.assert_allclose(spline(old)(grid,order),spline(new)(grid,order),atol=1e-10)
    profile=new['descent_profile']
    assert height_state(profile,profile['start']+profile['duration'],.08)[0]<.08
    assert new['hold_rotation'] is None
    np.testing.assert_allclose(new['velocities'][-1],0.,atol=1e-10)
    assert calls and new['generation']==1
    assert 'fixed_interception' not in new and 'descent_timing' not in new


def test_low_height_freezes_rotation_and_collision_failure_rejects(policy,synthetic_planner,monkeypatch):
    module,_=synthetic_planner
    c,old,obj=replacement_inputs(policy)
    old['descent_profile']=dict(start=0.,duration=.2,height0=.02,height1=.02,velocity0=0.)
    new=compute_replacement(c,old,obj,np.array(old['velocity']),100.,.8,1)
    np.testing.assert_allclose(new['hold_rotation'],np.eye(3))
    moved=obj.copy();moved[:3,:3]=Rotation.from_euler('z',90,degrees=True).as_matrix()
    pose,feed=target(np.array(old['initial_pose']),moved,np.array(old['velocity']),np.eye(4),.08,.1,1.,
                     new['descent_profile'],0.,policy,True,new['hold_rotation'])
    np.testing.assert_allclose(pose[:3,:3],np.eye(3));np.testing.assert_allclose(feed[3:],0.)
    def blocked(*args):raise ValueError('synthetic collision')
    monkeypatch.setattr(module,'checked_clearances',blocked)
    with pytest.raises(ValueError,match='synthetic collision'):
        compute_replacement(c,old,obj,np.array(old['velocity']),100.,.8,1)


def test_closure_rejects_missing_observation_or_future_exit(policy):
    assert closure_ok(policy,.019,.014,.6,True,True)
    assert not closure_ok(policy,.019,.014,.6,True,False)
    assert not closure_ok(policy,.019,.014,.6,False,True)
    assert not closure_ok(policy,.019,.014,.5,True,True)


def test_new_observation_updates_x_and_rotation_without_losing_prefix(policy,synthetic_planner):
    c,old,obj=replacement_inputs(policy)
    c['_descent_admitted']=False
    fresh=obj.copy();fresh[0,3]+=.006
    fresh[:3,:3]=Rotation.from_euler('z',30,degrees=True).as_matrix()
    new=compute_replacement(c,old,fresh,np.array(old['velocity']),100.1,.8,2)
    np.testing.assert_allclose(new['source_object'],fresh)
    grid=np.linspace(0,.8,51)
    np.testing.assert_allclose(spline(new)(grid),spline(old)(grid),atol=1e-10)
    later=spline(new)(1.6)
    assert later[14]>old['path'][0][14]
    assert later[19]>old['path'][0][19]
    assert new['descent_profile'] is None
