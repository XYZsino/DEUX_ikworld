"""Direct feedback validity must not depend on the BT observation relay."""
from types import SimpleNamespace
import numpy as np
import pytest
pytest.importorskip('rclpy')
from sensor_msgs.msg import JointState
from deux_task_msgs.msg import StateObservation
from deux_task_core.model import ARM_NAMES, HAND_NAMES
from deux_task_runtime.joint_feedback import JointFeedback
from deux_task_runtime.common import ros_time, RuntimeNode


def test_direct_feedback_order_staleness_malformed_and_clock_reset():
    now = [10.]
    node = SimpleNamespace(now=lambda: now[0], config={'max_joint_age':.1},
                           create_subscription=lambda *args: None)
    cache = JointFeedback(node)
    def send(side, stamp, value=0.):
        names = ARM_NAMES if side=='arm' else HAND_NAMES
        msg = JointState(name=list(names),position=[value]*len(names))
        msg.header.stamp=ros_time(stamp)
        cache.receive(side,msg)
    send('arm',10.); send('hand',10.)
    assert len(cache.measured())==28
    send('arm',9.99,1.)
    np.testing.assert_array_equal(cache.measured(),np.zeros(28))
    now[0]=10.02; send('arm',10.02,float('nan'))
    with pytest.raises(ValueError,match='arm'):cache.measured()
    send('arm',10.01)  # An older valid sample cannot undo fresh invalid feedback.
    with pytest.raises(ValueError):cache.measured()
    now[0]=10.03; send('arm',10.03)
    assert len(cache.measured())==28
    now[0]=10.11
    with pytest.raises(ValueError,match='hand'):cache.measured()
    now[0]=1.
    with pytest.raises(ValueError):cache.measured()
    assert not cache.samples
    send('arm',1.); send('hand',1.)
    assert len(cache.measured())==28


def test_bt_summary_has_no_joint_arrays_and_retains_age_guard():
    obs=StateObservation(joints_valid=True)
    obs.header.stamp=ros_time(10.)
    obs.joints_acquired_at=ros_time(9.95)
    node=SimpleNamespace(observation=obs,now=lambda:10.,
        config={'max_observation_age':.3,'max_joint_age':.1})
    RuntimeNode.robot_ready(node)
    assert not {'joints','arm_state','hand_state'} & set(obs.get_fields_and_field_types())
    obs.joints_acquired_at=ros_time(9.8)
    with pytest.raises(ValueError,match='stale'):RuntimeNode.robot_ready(node)
    obs.joints_acquired_at=ros_time(9.95);obs.joints_valid=False
    obs.joints_reason='hand missing'
    with pytest.raises(ValueError,match='hand missing'):RuntimeNode.robot_ready(node)
