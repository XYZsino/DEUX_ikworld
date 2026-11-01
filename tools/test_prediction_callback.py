import copy
import pytest
from test_prediction_hold import fixture,m

def test_planning_gap_uses_latest_received_valid_sample(monkeypatch):
 n,t,d=fixture(monkeypatch)
 m.MovingTrial.tracked_object(n)
 # No execution-loop polling while the worker computes for a second.
 for at in [.2,.4,.6]:
  t[0]=100.+at+.2
  msg=copy.deepcopy(n.objects);msg.header.stamp=100.+at
  m.MovingTrial.receive_prediction(n,msg)
 t[0]=101.29
 assert m.MovingTrial.tracked_object(n,True)[3]==100.6
 assert n.observation_held
 t[0]=101.601
 with pytest.raises(ValueError):m.MovingTrial.tracked_object(n,True)

def test_invalid_and_out_of_order_messages_do_not_renew_cache(monkeypatch):
 n,t,d=fixture(monkeypatch)
 m.MovingTrial.receive_prediction(n,n.objects)
 msg=copy.deepcopy(n.objects);msg.header.stamp=100.4;msg.detections[0].velocity_valid=False;t[0]=100.6
 m.MovingTrial.receive_prediction(n,msg)
 assert n._last_valid_prediction[3]==100.
 older=copy.deepcopy(msg);older.header.stamp=100.2;older.detections[0].velocity_valid=True
 m.MovingTrial.receive_prediction(n,older)
 assert n._last_valid_prediction[3]==100.
 assert n.objects.header.stamp==100.4
 t[0]=101.01
 with pytest.raises(ValueError):m.MovingTrial.tracked_object(n,True)
