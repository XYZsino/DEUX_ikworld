import numpy as np
import pytest
from deux_task_sim.visual_barcode_reader import atlas_surfaces,metrics,FIXTURE_VALUE


def test_printed_uv_point_maps_to_both_mesh_faces(tmp_path):
    mesh=tmp_path/'barcode.obj'
    mesh.write_text('v 0 0 .1\nv 1 0 .1\nv 0 1 .1\nv 0 0 -.1\nv 1 0 -.1\nv 0 1 -.1\n'
                    'vt 0 0\nvt 1 0\nvt 0 1\nvn 0 0 1\nvn 0 0 -1\n'
                    'f 1/1/1 2/2/1 3/3/1\nf 4/1/2 5/2/2 6/3/2\n')
    surfaces=atlas_surfaces(mesh,[.1,.1,0.],uv=[.2,.3])
    np.testing.assert_allclose(surfaces[0]['point'],[.1,.2,.1])
    np.testing.assert_allclose(surfaces[1]['point'],[.1,.2,-.1])
    np.testing.assert_allclose(surfaces[0]['normal'],-surfaces[1]['normal'])
    with pytest.raises(ValueError,match='expected front/back'):atlas_surfaces(mesh,[0,0,0],uv=[2.,2.])


def test_reader_keeps_distance_angle_and_front_face_contract():
    surface=dict(point=np.zeros(3),normal=np.array([0.,0.,1.]),horizontal=np.array([1.,0.,0.]))
    profile=dict(distance_m=[.05,.26],aim_error_deg_less_than=6.,pitch_abs_max_deg=60.,skew_abs_max_deg=60.)
    scanner=np.eye(4);scanner[2,3]=.12
    assert metrics(scanner,np.eye(4),surface,profile)['aligned']
    scanner[0,3]=.03
    assert not metrics(scanner,np.eye(4),surface,profile)['aligned']
    scanner[0,3]=0.;scanner[2,3]=.4
    assert not metrics(scanner,np.eye(4),surface,profile)['aligned']
    scanner[2,3]=-.12
    assert not metrics(scanner,np.eye(4),surface,profile)['aligned']
    assert int(FIXTURE_VALUE[-1])==(-sum(int(v)*(1 if i%2==0 else 3) for i,v in enumerate(FIXTURE_VALUE[:-1])))%10
