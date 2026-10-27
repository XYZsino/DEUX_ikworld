import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from deux_task_core.synthetic import fixture
from deux_task_core.planning import Planner
from deux_task_core.reader_alignment import check_posture,solve_fixed_wrist


def make_fixture(tmp_path):
    c=fixture(tmp_path);offset=np.eye(4);offset[:3,:3]=Rotation.from_euler('y',np.pi/2).as_matrix()
    c['left_ee_from_reader']=offset.tolist()
    c['barcode_fixed_wrist']=dict(joint_values=[0.,0.,0.],up_axis=[-1.,0.,0.],distance_range_m=[.06,.24],preferred_min_distance_m=.08,max_tilt_deg=5.,max_path_tilt_deg=15.,max_aim_deg=3.,max_view_deg=55.,start_tolerance_rad=.03)
    p=Planner(c);q=np.zeros(28);q[14]=.1
    label=np.eye(4);label[:3,:3]=Rotation.from_euler('y',-np.pi/2).as_matrix();label[:3,3]=p.robot.ee_fk('left',q)[:3,3]+[.12,.01,0.]
    return c,p,q,label


def test_fixed_wrist_scan_aims_without_changing_wrist_or_other_arm(tmp_path):
    c,p,q,label=make_fixture(tmp_path)
    data=p.plan('AlignBarcode',q,np.eye(4),barcode_pose=label)
    np.testing.assert_allclose(data['path'][:,4:7],0.,atol=1e-12)
    np.testing.assert_allclose(data['path'][:,14:21],np.broadcast_to(q[14:21],data['path'][:,14:21].shape),atol=1e-12)
    s=p.robot.ee_fk('left',data['path'][-1])@np.array(c['left_ee_from_reader']);d=label[:3,3]-s[:3,3];d/=np.linalg.norm(d)
    assert np.rad2deg(np.arccos(np.clip(s[:3,2]@d,-1,1)))<3.
    for row in data['path']:check_posture(p.robot,row,c,command=True)


def test_fixed_wrist_rejects_displaced_wrist_and_unreachable_barcode(tmp_path):
    c,p,q,label=make_fixture(tmp_path);q[4]=.1
    with pytest.raises(ValueError,match='ready angles'):solve_fixed_wrist(p.robot,q,label,c)
    q[4]=0.;label[0,3]+=3.
    with pytest.raises(ValueError,match='unreachable'):solve_fixed_wrist(p.robot,q,label,c)
    q[1]=.4
    with pytest.raises(ValueError,match='tilt'):check_posture(p.robot,q,c)
