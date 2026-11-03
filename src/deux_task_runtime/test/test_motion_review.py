import json
from types import SimpleNamespace
import numpy as np
import pytest
from deux_task_runtime.motion_review import Ticket,geometry,preview
from deux_task_core.synthetic import fixture
from deux_task_core.model import RobotModel
from deux_task_msgs.msg import MotionPlan


def test_ticket_is_bound_to_request_plan_and_digest_and_single_use():
    t=Ticket('p','hash',10,now=0)
    data=dict(request_id=t.request_id,plan_id='p',digest='hash',decision='approve')
    for key in ('request_id','plan_id','digest'):
        assert not t.decide(dict(data,**{key:'wrong'}),now=1)
    assert not t.decide(dict(data,decision='auto'),now=1)
    assert t.decide(data,now=1)
    assert not t.decide(dict(data,decision='reject'),now=2)


def test_expired_and_replaced_ticket_reject_previous_approval():
    a=Ticket('p','hash',1,now=0);b=Ticket('p','hash',1,now=0)
    data=dict(request_id=a.request_id,plan_id='p',digest='hash',decision='approve')
    assert not a.decide(data,now=1)
    assert not b.decide(data,now=.5)


def test_preview_uses_real_fk_and_bounded_path_samples(tmp_path):
    c=fixture(tmp_path);robot=RobotModel(c['robot']);q=np.zeros((120,28));q[:,0]=np.linspace(0,.1,120)
    p=MotionPlan(plan_id='p',stage='Prepare')
    d=preview(robot,p,q[0],np.linspace(0,2,120),q)
    assert len(d['frames'])==60 and d['sample_count']==120
    assert d['current']==geometry(robot,q[0]) and d['target']==geometry(robot,q[-1])
    json.dumps(d,allow_nan=False)
    old=d['digest'];p.follows_conveyor=True
    d=preview(robot,p,q[0],np.linspace(0,2,120),q)
    assert not d['approvable'] and d['digest']!=old
