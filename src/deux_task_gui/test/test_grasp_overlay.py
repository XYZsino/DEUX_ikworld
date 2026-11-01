import numpy as np
from test_rgb_view import fixture
from deux_task_gui.rgb_view import grasp_pixels,render_rgb

def test_future_point_projection_and_invalid_state():
 obs,_=fixture();g=np.eye(4);g[:3,3]=[.1,0,1];a=g.copy();a[2,3]+=.1
 preview=dict(feasible=True,target_id='a',grasp_pose=g.tolist(),approach_pose=a.tolist())
 pixels=grasp_pixels(obs,preview,'a');assert np.allclose(pixels[0],[56,36])
 _,meta=render_rgb(obs,None,'a',grasp_preview=preview);assert meta['grasp_preview_visible']
 assert grasp_pixels(obs,preview,'other') is None
 preview['expired']=True;assert grasp_pixels(obs,preview,'a') is None
 preview['expired']=False;preview['feasible']=False;assert grasp_pixels(obs,preview,'a') is None
 preview['feasible']=True;obs.camera_transform_valid=False;assert grasp_pixels(obs,preview,'a') is None
