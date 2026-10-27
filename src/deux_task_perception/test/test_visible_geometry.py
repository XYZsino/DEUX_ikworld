import numpy as np
import pytest
from deux_task_perception.visible_geometry import workspace_roi, surface_bounds


P=[100.,0.,50.,0.,0.,100.,50.,0.,0.,0.,1.,0.]


def test_workspace_projects_to_image_and_clips_to_visible_region():
    region=dict(min=[-.2,-.1,1.],max=[.3,.2,1.1])
    np.testing.assert_allclose(workspace_roi(region,np.eye(4),P,100,100),[30,40,80,70])
    region['min'][0]=-2.
    assert workspace_roi(region,np.eye(4),P,100,100)[0]==0.
    region['min'][2]=-.1
    with pytest.raises(ValueError,match='camera plane'):workspace_roi(region,np.eye(4),P,100,100)


def test_depth_bounds_require_mask_and_depth_and_reject_spikes():
    z=np.ones((100,100),np.float32);polygon=[20.,20.,70.,20.,70.,70.,20.,70.]
    a=surface_bounds(z,P,polygon,np.eye(4))
    z[40,40]=2.7
    b=surface_bounds(z,P,polygon,np.eye(4))
    np.testing.assert_allclose(a,b,atol=.001)
    with pytest.raises(ValueError):surface_bounds(z,P,[],np.eye(4))
    z[:]=np.nan
    with pytest.raises(ValueError,match='깊이'):surface_bounds(z,P,polygon,np.eye(4))
