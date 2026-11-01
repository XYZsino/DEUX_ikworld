import json
from urllib.request import Request,urlopen
from urllib.error import HTTPError
import pytest
from test_gui import Harness

def test_roi_persistence_auth_geometry_and_active_guard(tmp_path,monkeypatch):
 path=tmp_path/'roi.json';monkeypatch.setenv('DEUX_TRACK_ROI_FILE',str(path));h=Harness()
 data=dict(width=640,height=360,points=[[100,100],[500,100],[500,300],[100,300]])
 def request(data=None,token=None):
  req=Request(h.url+'/api/tracking-roi',data=json.dumps(data).encode() if data else None,headers={'Authorization':'Bearer '+(token or h.gui.token),'Content-Type':'application/json'})
  return json.load(urlopen(req,timeout=2))
 try:
  h.spin(lambda:h.gui.snapshot().get('connected'))
  assert request()['roi'] is None
  with pytest.raises(HTTPError) as ex:request(data,'wrong')
  assert ex.value.code==401
  assert request(data)['roi']==data and json.loads(path.read_text())==data
  h.active=True;h.spin(lambda:h.gui.snapshot().get('status',{}).get('active'))
  with pytest.raises(HTTPError) as ex:request(data)
  assert ex.value.code==409
 finally:h.close()
