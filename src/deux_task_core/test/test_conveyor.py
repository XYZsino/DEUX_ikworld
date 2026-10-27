from types import SimpleNamespace
import numpy as np
import pytest
from deux_task_core.conveyor import plan_track_grasp, plan_conveyor, forecast
from deux_task_core.model import JOINT_NAMES,HAND_NAMES


class CartesianRobot:
    ee={'right':'right_grasp'}
    velocity=np.ones(28)*10
    acceleration=np.ones(28)*50
    ik_calls=0
    def ik(self,side,target,seed,**kwargs):
        self.ik_calls+=1
        q=seed.copy();q[14:17]=target[:3,3];return q
    def clearance(self,*args):return .1
    def fk(self,q):
        pose=np.eye(4);pose[:3,3]=q[14:17];return {'right_grasp':pose}


def setup_plan():
    offset=np.eye(4);offset[2,3]=.08
    cfg=dict(conveyor_tracking=dict(max_speed=.15,grasp_region=dict(min=[.4,-.5,.6],max=[.6,.3,.8]),
        acquisition_region=dict(min=[.4,-.5,.6],max=[.6,.6,.8]),boundary_margin=.015,max_plan_horizon=8.,durations=dict(Approach=1.5,Track=1.5,Descend=.8,Close=1.3)),
        grasp_object_from_right_ee=offset,approach_height=.07,hand_open=np.zeros(14),hand_closed=np.ones(14)*.2,
        max_target_drift=.025,sample_period=.02,obstacles=[],collision_margin=.005)
    q=np.zeros(28);q[14:17]=[.5,0,.9]
    obj=np.eye(4);obj[:3,3]=[.5,.15,.7]
    return SimpleNamespace(config=cfg,robot=CartesianRobot()),q,obj


def test_intercept_then_track_without_closing_early():
    planner,q,obj=setup_plan();v=np.array([0,-.08,0,0]);lead=.5
    data=plan_track_grasp(planner,q,obj,v,lead)
    ends=data['phase_end_times'];np.testing.assert_allclose(ends,[1.5,2.3,3.6])
    ts=data['times'];path=data['path'];hands=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    # Arrival already matches the moving target. Track retains vertical clearance
    # and velocity before descent or finger closure starts.
    track=(ts>ends[0]-.04)&(ts<=ends[0])
    np.testing.assert_allclose(path[track,15],obj[1,3]+v[1]*(lead+ts[track]),atol=5e-4)
    np.testing.assert_allclose(path[track,16],.85,atol=5e-4)
    np.testing.assert_allclose(path[ts<=ends[1]][:,hands],0,atol=2e-4)
    np.testing.assert_allclose(path[-1,hands],.2,atol=1e-10)
    np.testing.assert_allclose(path[0],q,atol=1e-10)


def test_future_exit_rejected_before_motion():
    planner,q,obj=setup_plan();obj[1,3]=-.3
    with pytest.raises(ValueError,match='exits grasp region'):
        plan_track_grasp(planner,q,obj,[0,-.08,0,0],.5)


def test_approach_intercepts_future_position_then_tracking_uses_new_pose():
    planner,q,obj=setup_plan();velocity=np.array([0,-.08,0,0]);lead=.5
    approach=plan_conveyor(planner,'Approach',q,obj,velocity,lead)
    arrival=forecast(obj,velocity,lead+approach['times'][-1])@planner.config['grasp_object_from_right_ee']
    arrival[2,3]+=planner.config['approach_height']
    np.testing.assert_allclose(approach['path'][-1,14:17],arrival[:3,3],atol=1e-8)
    # A new observed pose can deviate from the previous forecast; the tracking
    # path must use it instead of replaying the old acquisition snapshot.
    new=forecast(obj,velocity,lead+approach['times'][-1]);new[0,3]+=.01
    track=plan_track_grasp(planner,approach['path'][-1],new,velocity,lead)
    index=np.argmin(abs(track['times']-track['phase_end_times'][0]))
    assert track['path'][index,14]==pytest.approx(new[0,3],abs=.001)


def test_upstream_approach_allowed_only_if_future_intercept_is_inside():
    planner,q,obj=setup_plan();obj[1,3]=.38
    data=plan_conveyor(planner,'Approach',q,obj,[0,-.08,0,0],.5)
    assert data['path'][-1,15]<planner.config['conveyor_tracking']['grasp_region']['max'][1]
    with pytest.raises(ValueError,match='outside right grasp region'):
        plan_track_grasp(planner,q,obj,[0,-.08,0,0],.5)
    with pytest.raises(ValueError,match='exits right grasp region'):
        plan_conveyor(planner,'Approach',q,obj,[0,.08,0,0],.5)


def test_slow_finger_retimes_closure_without_extending_interception():
    planner,q,obj=setup_plan()
    planner.robot.acceleration=np.ones(28)*50
    hands=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    planner.robot.acceleration[hands]=.3
    data=plan_track_grasp(planner,q,obj,[0,-.08,0,0],.5)
    np.testing.assert_allclose(data['phase_end_times'][:2],[1.5,2.3])
    assert data['phase_end_times'][2]>3.6
    velocity=np.diff(data['path'],axis=0)/np.diff(data['times'])[:,None]
    acceleration=np.diff(np.vstack([np.zeros(28),velocity,np.zeros(28)]),axis=0)/np.r_[np.diff(data['times']),np.diff(data['times'])[-1]][:,None]
    assert np.max(abs(acceleration)/planner.robot.acceleration)<=1.


def test_tracking_velocity_uses_differential_kinematics_not_ik_residual():
    from deux_task_core.conveyor import tracking_joint_velocity
    robot=CartesianRobot();q=np.zeros(28)
    def unreliable_ik(*args,**kwargs):raise AssertionError('IK must not be differentiated')
    robot.ik=unreliable_ik
    velocity=tracking_joint_velocity(robot,q,[0,-.08,0],[0,0,0])
    dt=.001
    measured=(robot.fk(q+velocity*dt)['right_grasp'][:3,3]-robot.fk(q)['right_grasp'][:3,3])/dt
    np.testing.assert_allclose(measured,[0,-.08,0],atol=1e-9)
    with pytest.raises(ValueError,match='cannot match conveyor velocity'):
        tracking_joint_velocity(robot,q,[0,-.08,0],[0,0,.1])


def test_direct_descent_has_no_track_and_preserves_half_grip_until_close():
    planner,q,obj=setup_plan();cfg=planner.config
    cfg['conveyor_tracking']['durations'].update(Track=0.,Close=1.)
    hands=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    cfg['hand_pregrasp']=np.ones(14)*.1;q[hands]=.1
    q[14:17]=[.5,.09,.85]
    data=plan_track_grasp(planner,q,obj,[0.,-.08,0.,0.],.5)
    ts=data['times'];path=data['path'];ends=data['phase_end_times']
    assert ends[0]==0.
    np.testing.assert_allclose(path[0],q)
    assert path[1,16]<q[16]  # immediately descending, no fixed Track dwell
    np.testing.assert_allclose(path[ts<=ends[1]][:,hands],.1,atol=1e-9)
    np.testing.assert_allclose(path[-1,hands],.2)
    velocity=np.diff(path,axis=0)/np.diff(ts)[:,None]
    acceleration=np.diff(np.vstack([np.zeros(28),velocity,np.zeros(28)]),axis=0)/np.r_[np.diff(ts),np.diff(ts)[-1]][:,None]
    assert np.max(abs(velocity)/planner.robot.velocity)<=1.
    assert np.max(abs(acceleration)/planner.robot.acceleration)<=1.


def test_half_grip_reduces_dynamics_limited_closure_time():
    planner,q,obj=setup_plan();hands=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    planner.robot.acceleration=np.ones(28)*50;planner.robot.acceleration[hands]=.3
    opened=plan_track_grasp(planner,q,obj,[0,-.08,0,0],.5)
    planner.config['hand_pregrasp']=np.ones(14)*.1;q[hands]=.1
    half=plan_track_grasp(planner,q,obj,[0,-.08,0,0],.5)
    assert half['phase_end_times'][-1]<opened['phase_end_times'][-1]


def entry_setup():
    planner,_,obj=setup_plan();c=planner.config;cfg=c['conveyor_tracking']
    cfg.update(start_lead=.4,intercept_wait_s=1.,grasp_region=dict(min=[.45,-.48,.68],max=[.6,.12,.82]))
    cfg['durations'].update(Approach=1.5,Track=0.,Descend=.8,Close=1.)
    obj[:3,3]=[.52,0.,.75]
    return c,obj


def test_entry_uses_remaining_phases_instead_of_fixed_seven_seconds():
    from deux_task_core.conveyor import grasp_entry_prediction,inside
    c,obj=entry_setup();v=[0,-.08,0,0]
    result=grasp_entry_prediction(c,obj,v,0.)
    assert result['remaining_s']==pytest.approx(5.1)
    assert result['failure'] is None
    assert not inside(forecast(obj,v,7.)[:3,3],c['conveyor_tracking']['grasp_region'],.015)
    # Old installations may retain the obsolete key; it cannot affect the gate.
    c['conveyor_tracking']['entry_lookahead']=100.
    assert grasp_entry_prediction(c,obj,v,0.)['failure'] is None


def test_entry_accounts_for_observation_age_and_changed_motion_times():
    from deux_task_core.conveyor import grasp_entry_prediction
    c,obj=entry_setup();v=[0,-.08,0,0]
    assert grasp_entry_prediction(c,obj,v,.8)['failure']=='completion'
    c['conveyor_tracking']['durations']['Close']=2.
    result=grasp_entry_prediction(c,obj,v,0.)
    assert result['remaining_s']==pytest.approx(6.1) and result['failure']=='completion'
    c['conveyor_tracking']['intercept_wait_s']=0.
    c['conveyor_tracking']['durations']['Track']=.3
    assert grasp_entry_prediction(c,obj,v,.2)['completion_s']==pytest.approx(5.6)


def test_entry_still_rejects_upstream_arrival_and_outside_acquisition():
    from deux_task_core.conveyor import grasp_entry_prediction
    c,obj=entry_setup();obj[1,3]=.4
    assert grasp_entry_prediction(c,obj,[0,-.08,0,0],0.)['failure']=='arrival'
    obj[0,3]=1.
    assert grasp_entry_prediction(c,obj,[0,-.08,0,0],0.)['failure']=='acquisition'


@pytest.mark.parametrize('stage',['Approach','Descend','Close','TrackGrasp'])
def test_ik_workspace_plans_outside_old_box_and_keeps_collision_checks(stage):
    planner,q,obj=setup_plan();obj[0,3]=.65;q[14]=.65
    plan=lambda: (plan_track_grasp(planner,q,obj,[0.,-.08,0.,0.],.5) if stage=='TrackGrasp'
                  else plan_conveyor(planner,stage,q,obj,[0.,-.08,0.,0.],.5))
    with pytest.raises(ValueError,match='region'):plan()
    planner.config['conveyor_tracking']['workspace_policy']='ik'
    data=plan()
    assert planner.robot.ik_calls>0
    np.testing.assert_allclose(data['path'][:,14],.65)
    assert data['minimum_clearance']==.1
    def collision(*args):raise ValueError('collision clearance')
    planner.robot.clearance=collision
    with pytest.raises(ValueError,match='collision clearance'):plan()


@pytest.mark.parametrize('stage',['Approach','TrackGrasp'])
def test_ik_workspace_still_rejects_unsolved_poses(stage):
    planner,q,obj=setup_plan();obj[0,3]=.65
    planner.config['conveyor_tracking']['workspace_policy']='ik'
    def unreachable(*args,**kwargs):raise ValueError('IK unreachable')
    planner.robot.ik=unreachable
    with pytest.raises(ValueError,match='IK'):
        if stage=='Approach':plan_conveyor(planner,stage,q,obj,[0.,-.08,0.,0.],.5)
        else:plan_track_grasp(planner,q,obj,[0.,-.08,0.,0.],.5)


def test_ik_workspace_rendezvous_and_entry_have_no_box_gate():
    from deux_task_core.conveyor import rendezvous_ready,grasp_entry_prediction
    c,obj=entry_setup();obj[0,3]=.65
    cfg=c['conveyor_tracking'];cfg.update(intercept_trigger_distance_m=.025,intercept_height_tolerance_m=.02)
    velocity=[0.,-.08,0.,0.]
    ee=forecast(obj,velocity,cfg['start_lead'])@c['grasp_object_from_right_ee'];ee[2,3]+=c['approach_height']
    assert not rendezvous_ready(obj,velocity,0.,ee,c)
    cfg['workspace_policy']='ik'
    assert rendezvous_ready(obj,velocity,0.,ee,c)
    assert grasp_entry_prediction(c,obj,velocity,0.)['failure'] is None
    ee[0,3]+=.1
    assert not rendezvous_ready(obj,velocity,0.,ee,c) # hand proximity still required
