import numpy as np
import pytest
from deux_task_perception.perception_core import filter_above_plane

def test_plane_filter_uses_base_frame_and_keeps_original_coordinates():
    points=np.array([[0.,0.,-.03],[0.,0.,0.],[0.,0.,.03]])
    transform=np.eye(4);transform[2,3]=.8
    before=points.copy();diag={}
    result=filter_above_plane(points,transform,[0.,0.,1.,-.8],.01,diag)
    np.testing.assert_array_equal(result,points[2:])
    np.testing.assert_array_equal(points,before)
    assert diag['table_removed_points']==2 and diag['table_retained_points']==1

def test_tilted_plane_uses_signed_distance_not_absolute_z():
    n=np.array([.6,0.,.8]);points=np.array([n*.03,-n*.03,[0.,0.,0.]])
    np.testing.assert_allclose(filter_above_plane(points,np.eye(4),[*n,0.],.01),points[:1])

def test_empty_and_invalid_plane():
    assert filter_above_plane(np.empty((0,3)),np.eye(4),[0.,0.,1.,0.],.01).shape==(0,3)
    with pytest.raises(ValueError):filter_above_plane(np.zeros((1,3)),np.eye(4),[0.,0.,2.,0.],.01)
