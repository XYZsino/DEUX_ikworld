from types import SimpleNamespace
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from deux_task_runtime.leader_grip_adapter import LeaderGripHandAdapter

def test_continuous_placement_does_not_open_at_arm_start():
 adapter=LeaderGripHandAdapter.__new__(LeaderGripHandAdapter);emitted=[]
 adapter.node=SimpleNamespace(config={'hand_command_only':True,'placement':{'continuous_motion':True}},get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time())))
 adapter.guard=lambda:None;adapter.stage='PlaceApproach';adapter.binary_goal={'left':1.,'right':0.};adapter.virtual_right=False;adapter.tag='test';adapter.last_command={}
 adapter.command_pubs={s:SimpleNamespace(publish=lambda m:emitted.append((m.side,m.grasp))) for s in ('left','right')}
 for value in (1.,1.,0.,0.):adapter.command(np.zeros(14),placement_right_fraction=value)
 assert [v for side,v in emitted if side=='right']==[1.,1.,0.,0.]
 assert [v for side,v in emitted if side=='left']==[1.,1.,1.,1.]
 adapter.stage='Prepare'
 with pytest.raises(ValueError,match='invalid continuous'):adapter.command(np.zeros(14),placement_right_fraction=0.)
