import json
import time
from urllib.error import HTTPError
import pytest
from std_msgs.msg import String
from rclpy.qos import QoSProfile,DurabilityPolicy
from test_gui import Harness


def test_http_review_requires_auth_fresh_matching_preview():
    h=Harness();decisions=[]
    pub=h.server.create_publisher(String,'/task/motion_preview',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    sub=h.server.create_subscription(String,'/task/motion_decision',lambda m:decisions.append(json.loads(m.data)),10)
    data=dict(request_id='request1',plan_id='plan1',digest='digest1',status='pending',
              current={},target={},frames=[],approvable=True)
    try:
        pub.publish(String(data=json.dumps(data)))
        h.spin(lambda:h.gui.motion_preview is not None)
        with pytest.raises(HTTPError) as err:h.request('motion_approve','request1',token='bad')
        assert err.value.code==401
        h.request('motion_approve','old-request');h.spin(lambda:'다시 확인' in h.gui.message)
        assert not decisions
        h.request('motion_approve','request1');h.spin(lambda:len(decisions)==1)
        assert decisions[0]==dict(request_id='request1',plan_id='plan1',digest='digest1',decision='approve')
        h.gui.on_preview(String(data=json.dumps(data)));h.gui.preview_received=time.monotonic()-2
        h.request('motion_approve','request1');h.spin(lambda:'다시 확인' in h.gui.message)
        assert len(decisions)==1
        data.update(request_id='request2',approvable=False)
        h.gui.on_preview(String(data=json.dumps(data)))
        h.request('motion_approve','request2');h.spin(lambda:'이동 컨베이어' in h.gui.message)
        assert len(decisions)==1
        h.request('motion_reject','request2');h.spin(lambda:len(decisions)==2)
        assert decisions[-1]['decision']=='reject'
    finally:h.close()
