import json
from pathlib import Path
import numpy as np
import pytest
from deux_task_core.tracking_policy import entry_prediction

def test_wall_time_and_latency_and_boundaries():
 p=json.loads(Path('calibration/tracking_grasp_policy.json').read_text())
 o=np.eye(4);o[:3,3]=[.6,.13,.97];v=np.array([0.,-.1,0.,0.])
 yes,point=entry_prediction(p,o,v,100.,100.2)
 assert yes;assert point[1]==pytest.approx(.01)
 assert not entry_prediction(p,o,v,100.,100.)[0]
 o[1,3]=-.15
 assert not entry_prediction(p,o,v,100.,100.2)[0]
 o[0,3]=.671;o[1,3]=.0
 assert not entry_prediction(p,o,v,100.,100.2)[0]
 with pytest.raises(ValueError):entry_prediction(p,o,v,101.,100.)
