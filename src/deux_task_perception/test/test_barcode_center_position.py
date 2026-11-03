import numpy as np
import pytest
from deux_task_perception.barcode_geometry import center_position

P=[100.,0.,2.,0.,0.,100.,2.,0.,0.,0.,1.,0.]
def test_mean_depth_center_ray_and_invalid_pixels():
    d=np.full((5,5),np.nan);d[1:4,1:4]=np.arange(1,10).reshape(3,3)/10
    d[1,1]=np.nan
    pos,fraction=center_position(d,P,[1,1,3,3])
    np.testing.assert_allclose(pos,[0,0,.55]);assert fraction==8/9

def test_no_depth_and_edge():
    with pytest.raises(ValueError,match='no valid depth'):
        center_position(np.zeros((5,5)),P,[1,1,3,3])
    pos,fraction=center_position(np.ones((5,5)),P,[0,0,.2,.2])
    np.testing.assert_allclose(pos,[-.019,-.019,1]);assert fraction==1
