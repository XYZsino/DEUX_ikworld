from types import SimpleNamespace as S
import pytest
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from deux_task_runtime.hand_adapter import HandAdapter
from deux_task_runtime.execution_node import ExecutionNode
from deux_task_runtime.common import ros_time


def test_feedback_age_reports_which_hand_without_discarding_fresh_side():
    adapter=HandAdapter.__new__(HandAdapter)
    adapter.node=S(now=lambda:100.,config={'max_joint_age':.2})
    adapter.raw={'left':(Header(stamp=ros_time(99.99)),{}),'right':(Header(stamp=ros_time(99.7)),{})}
    with pytest.raises(ValueError,match=r'HAND_FEEDBACK_STALE.*right: age=0.3000s'):
        adapter.measured_hand()
    assert 'left' in adapter.raw


def test_readonly_health_requires_feedback_and_no_pending_arm_action():
    checks=[]
    def checked(name):return lambda:checks.append(name)
    node=S(hardware_gate=checked('hardware'),arm=S(validate=checked('idle'),measured_arm=checked('arm_feedback')),hand=S(guard=checked('hand_guard')))
    result=ExecutionNode.execution_health(node,Trigger.Request(),Trigger.Response())
    assert result.success and checks==['hardware','idle','arm_feedback','hand_guard']
    def pending():raise ValueError('previous arm execution/cancellation pending')
    node.arm.validate=pending;checks.clear()
    result=ExecutionNode.execution_health(node,Trigger.Request(),Trigger.Response())
    assert not result.success and 'pending' in result.message and checks==['hardware']
