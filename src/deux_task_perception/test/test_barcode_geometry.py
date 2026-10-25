import numpy as np
import pytest
from deux_task_perception.barcode_geometry import surface, associated_bag

P = [100.,0.,19.5,0.,0.,100.,14.5,0.,0.,0.,1.,0.]


def test_plane_normal_faces_camera_and_missing_background_rejected():
    depth = np.full((30,40), .5)
    pose, rmse, fraction = surface(depth,P,[5,5,34,24])
    assert rmse < 1e-8 and fraction == 1.
    assert pose[2,2] < -.99 and np.linalg.det(pose[:3,:3]) == pytest.approx(1.)
    depth[:] = np.nan
    with pytest.raises(ValueError): surface(depth,P,[5,5,34,24])
    depth[:] = .5; depth[:,20:] = .8
    with pytest.raises(ValueError): surface(depth,P,[5,5,34,24])


def test_unique_bag_association_and_wrong_target_rejection():
    bag = [0,0,39,29]; label=[10,10,25,20]
    associated_bag(label,[bag],[0,0,.5],P)
    with pytest.raises(ValueError): associated_bag(label,[bag,bag],[0,0,.5],P)
    with pytest.raises(ValueError): associated_bag(label,[bag],[1,0,.5],P)
