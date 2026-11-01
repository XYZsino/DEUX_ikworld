from types import SimpleNamespace
import pytest
from deux_task_runtime.align_hand_pulse import pulse_fraction
from deux_task_runtime.leader_grip_adapter import LeaderGripHandAdapter


def test_partial_open_close_timing():
    for t, expected, done in [(0,1,False),(.25,.85,False),(.5,.7,False),(.75,.85,False),(1,1,True),(2,1,True)]:
        fraction, completed = pulse_fraction(t,.3,.5)
        assert fraction == pytest.approx(expected)
        assert completed == done


def test_alignment_pulse_only_commands_left_and_requires_guard():
    from builtin_interfaces.msg import Time
    emitted=[]
    node=SimpleNamespace(get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time())))
    adapter=SimpleNamespace(stage='AlignBarcode',guard=lambda:None,node=node,tag='test',last_command={},
                            command_pubs={'left':SimpleNamespace(publish=emitted.append)})
    command=LeaderGripHandAdapter.command_align_left_fraction
    command(adapter,.7)
    assert len(emitted)==1 and emitted[0].side=='left'
    assert emitted[0].grasp==pytest.approx(.7)
    adapter.stage='Prepare'
    with pytest.raises(ValueError):command(adapter,.7)
    adapter.stage='AlignBarcode'
    def reject():raise ValueError('mode lost')
    adapter.guard=reject
    with pytest.raises(ValueError,match='mode lost'):command(adapter,1.)
    assert len(emitted)==1


def test_ready_hold_then_single_close():
    from deux_task_runtime.align_hand_pulse import close_fraction, stage_left_fraction
    for stage in ('ReadyPose','Prepare','Approach','Descend','TrackGrasp','Close','Lift','ScanPose','AlignBarcode'):
        assert stage_left_fraction(stage,.3)==.3
    assert stage_left_fraction('Place',.3)==1.
    for t,expected,done in [(0,.3,False),(.25,.65,False),(.5,1,True),(1,1,True)]:
        f,finished=close_fraction(t,.3,.5)
        assert f==pytest.approx(expected)
        assert finished==done


def test_alignment_ramp_sends_one_left_command_without_reopening():
    from builtin_interfaces.msg import Time
    node=SimpleNamespace(config={'hand_command_only':True},get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time())))
    emitted=[]
    adapter=SimpleNamespace(node=node,guard=lambda:None,stage='AlignBarcode',binary_goal={'left':0.,'right':1.},
        virtual_right=False,tag='test',last_command={},command_pubs={s:SimpleNamespace(publish=emitted.append) for s in ('left','right')})
    from deux_task_runtime.align_hand_pulse import close_fraction
    command=LeaderGripHandAdapter.command
    for elapsed in (0.,.25,.5,1.):
        f,_=close_fraction(elapsed,0.,.5)
        command(adapter,[0.]*14,align_left_fraction=f)
    assert [m.grasp for m in emitted if m.side=='left']==[0.,.5,1.,1.]
    assert [m.grasp for m in emitted if m.side=='right']==[1.,1.,1.,1.]
    adapter.stage='ReadyPose'
    with pytest.raises(ValueError):command(adapter,[0.]*14,align_left_fraction=.5)
