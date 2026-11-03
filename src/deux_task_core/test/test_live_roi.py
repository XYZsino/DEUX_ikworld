import numpy as np
from deux_task_core.live_continuous_preview import roi_contains

def test_future_roi_exit_and_behind_camera():
 g=dict(camera=np.eye(4).tolist(),projection=[100,0,50,0,0,100,50,0,0,0,1,0],width=100,height=100,roi=dict(width=100,height=100,points=[[10,10],[90,10],[90,90],[10,90]]))
 assert roi_contains(g,[[0,0,1],[.2,0,1]])
 assert not roi_contains(g,[[0,0,1],[.5,0,1]])
 assert not roi_contains(g,[[0,0,-1]])
