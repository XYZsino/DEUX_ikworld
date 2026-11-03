import time
from types import SimpleNamespace
import pytest
from rcl_interfaces.msg import ParameterEvent,Parameter
from deux_hand_msgs.msg import HandUiMessage
from deux_task_runtime.leader_grip_adapter import LeaderGripHandAdapter


def adapter():
    a=LeaderGripHandAdapter.__new__(LeaderGripHandAdapter)
    now=time.monotonic()
    a.node=SimpleNamespace(config={'max_joint_age':.2},count_publishers=lambda _:0)
    a.mapping={s:{'leader_mode':3} for s in ('left','right')}
    a.profile_state={s:(now-3600,True) for s in a.mapping}
    a.profile_generation={s:0 for s in a.mapping};a.feedback_seen={s:now for s in a.mapping}
    a.mode_state={s:(now,HandUiMessage(leader_mode=3,teleop_mode=0,run_flag=True,stop_flag=False)) for s in a.mapping}
    a.virtual_right=False;a.estop=False;a.last_external={}
    a.command_pubs={s:SimpleNamespace(get_subscription_count=lambda:1) for s in a.mapping}
    a.measured_hand=lambda:None
    return a


def test_old_verified_profiles_remain_valid_with_live_feedback():
    a=adapter();a.guard()
    a.client=SimpleNamespace(service_is_ready=lambda:pytest.fail('unnecessary profile refresh'))
    a.pending=None;a.refresh_profile()


def test_feedback_loss_invalidates_cache_and_reconnect_needs_verification():
    a=adapter();a.feedback_seen['right']-=3
    with pytest.raises(ValueError,match='unverified'):a.guard()
    a.feedback_seen['right']=time.monotonic()
    with pytest.raises(ValueError,match='unverified'):a.guard()


def test_mode_request_and_change_invalidate():
    a=adapter();a.mode_requested('right');assert 'right' not in a.profile_state
    a=adapter();a.receive_mode('left',HandUiMessage(leader_mode=1,run_flag=True));assert 'left' not in a.profile_state


def test_parameter_change_invalidates_both_shared_node_profiles():
    a=adapter();a.profile_event(ParameterEvent(node='/other',changed_parameters=[Parameter(name='leader_mode_3.target_deg')]))
    assert len(a.profile_state)==2
    a.profile_event(ParameterEvent(node='/deux_hand',changed_parameters=[Parameter(name='leader_mode_3.target_deg')]))
    assert not a.profile_state


def test_inflight_response_cannot_restore_invalidated_cache():
    from rclpy.parameter import Parameter as RP
    a=adapter();a.profile_state.clear()
    generation=dict(a.profile_generation);a.invalidate_profile('right')
    names=['hand_side'];future=SimpleNamespace(result=lambda:SimpleNamespace(values=[RP('hand_side',value='right').get_parameter_value()]))
    a.receive_profile(names,future,generation)
    assert 'right' not in a.profile_state


def test_brief_gap_preserves_static_profile_but_stale_feedback_still_blocks():
    a=adapter();a.feedback_seen['right']-=.3
    def stale():raise ValueError('hand feedback stale/missing')
    a.measured_hand=stale
    with pytest.raises(ValueError,match='feedback stale'):a.guard()
    assert a.profile_state['right'][1]
    a.measured_hand=lambda:None;a.feedback_seen['right']=time.monotonic();a.guard()

def test_stale_packet_does_not_invalidate_verified_profile():
    from deux_hand_msgs.msg import HandCtlMessage,MotorData
    a=adapter();a.raw={};a.feedback=lambda side,msg:None
    m=HandCtlMessage(side='right',motors=[MotorData(id=i+1,torque=True) for i in range(7)])
    a.receive_feedback('right',m,b'one')
    assert a.profile_state['right'][1] and 'right' not in a.raw
    a.receive_feedback('right',m,b'two')
    assert 'right' not in a.profile_state
    assert 'restarted' in a.profile_reason['right']

def test_torque_off_invalidates_profile_and_raw():
    from deux_hand_msgs.msg import HandCtlMessage,MotorData
    a=adapter();a.raw={'right':object()}
    a.receive_feedback('right',HandCtlMessage(side='right',motors=[MotorData(id=1,torque=False)]))
    assert 'right' not in a.raw and 'right' not in a.profile_state


def test_endpoint_restart_invalidates_only_changed_side():
    a=adapter();a.feedback_publishers={};ids={'left':b'left-1','right':b'right-1'}
    for side in ids:a.mapping[side]['feedback_topic']=side
    a.node.get_publishers_info_by_topic=lambda topic:[SimpleNamespace(endpoint_gid=ids[topic])]
    a.check_feedback_publishers();assert len(a.profile_state)==2
    ids['right']=b'right-2';a.check_feedback_publishers()
    assert 'left' in a.profile_state and 'right' not in a.profile_state

def test_profile_request_timeout_allows_retry():
    from rclpy.task import Future
    a=adapter();a.profile_state.clear();a.pending=Future();a.pending_started=time.monotonic()-2
    removed=[];future=a.pending
    a.client=SimpleNamespace(remove_pending_request=removed.append,service_is_ready=lambda:False)
    a.refresh_profile()
    assert removed==[future] and future.cancelled() and a.pending is None
