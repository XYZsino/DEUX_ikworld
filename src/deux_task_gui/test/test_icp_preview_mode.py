import json
import pytest
from std_msgs.msg import String
from urllib.error import HTTPError
from test_gui import Harness

def test_preview_blocks_robot_commands():
 h=Harness()
 try:
  h.gui.on_icp_preview(String(data=json.dumps({'perception_only':True,'target_id':'bag'})))
  h.spin(lambda:h.gui.snapshot().get('perception_only'))
  for command in ('start','ready','restart'):
   with pytest.raises(HTTPError) as exc:h.request(command)
   assert exc.value.code==409
  assert not h.goals and h.ready_goals==0
 finally:h.close()
