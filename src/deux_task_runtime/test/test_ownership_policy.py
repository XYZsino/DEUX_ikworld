from types import SimpleNamespace
import pytest
from deux_task_runtime.common import RuntimeNode

def rig(policy="warn", calibrated=True, teleop="off", publishers=3):
    warnings=[]
    config=dict(synthetic=False, calibration_confirmed=calibrated,
                command_ownership_confirmed=False, command_ownership_policy=policy,
                require_state_status=True, allowed_state_modes=["standby"])
    node=SimpleNamespace(config=config, robot_status=dict(mode="standby",teleop=teleop),
        count_publishers=lambda topic: publishers if topic.endswith("joint_trajectory") else 1,
        get_logger=lambda: SimpleNamespace(warning=lambda message,**kwargs:warnings.append(message)))
    return node,warnings

def test_explicit_warn_allows_registered_publishers_without_claiming_ownership():
    node,warnings=rig()
    RuntimeNode.hardware_gate(node)
    assert len(warnings)==1 and "continuing" in warnings[0]
    assert node.config["command_ownership_confirmed"] is False

def test_default_policy_remains_reject():
    node,_=rig()
    del node.config["command_ownership_policy"]
    with pytest.raises(ValueError,match="ownership"):RuntimeNode.hardware_gate(node)

@pytest.mark.parametrize("case",["calibration","teleop","mode","status_lost","invalid_policy"])
def test_warning_policy_does_not_disable_other_gates(case):
    node,_=rig()
    if case=="calibration":node.config["calibration_confirmed"]=False
    if case=="teleop":node.robot_status["teleop"]="on"
    if case=="mode":node.robot_status["mode"]="packing"
    if case=="status_lost":node.count_publishers=lambda topic:0
    if case=="invalid_policy":node.config["command_ownership_policy"]="waarn"
    with pytest.raises(ValueError):RuntimeNode.hardware_gate(node)
