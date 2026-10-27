import numpy as np
import pytest
from deux_task_core.synthetic import fixture
from deux_task_core.model import RobotModel
from deux_task_core.planning import Planner, validate_scene, hermite
from deux_task_core.filter import PoseFilter


def test_urdf_fk_ik_limits_and_collision(tmp_path):
    c = fixture(tmp_path)
    r = RobotModel(c["robot"])
    q = np.zeros(28)
    q[:7] = [0.1, -0.2, 0.1, 0.1, -0.1, 0.2, 0.1]
    target = r.fk(q)[r.ee["left"]]
    solved = r.ik("left", target, np.zeros(28))
    np.testing.assert_allclose(r.fk(solved)[r.ee["left"]], target, atol=0.003)
    r.clearance(q, [], 0.01)
    center = r.fk(q)[r.ee["left"]][:3, 3]
    with pytest.raises(ValueError, match="collision"):
        r.clearance(
            q, [dict(min=(center - 0.1).tolist(), max=(center + 0.1).tolist())], 0.01
        )
    q[0] = 3
    with pytest.raises(ValueError, match="joint limit"):
        r.clearance(q, [], 0.01)


def test_scene_rejects_missing_installation_and_plan_hand_phase(tmp_path):
    c = fixture(tmp_path)
    validate_scene(c)
    planner = Planner(c)
    data = planner.plan("Close", np.zeros(28), np.eye(4))
    assert data["path"].shape[1] == 28 and len(data["times"]) > 2
    assert max(data["path"][-1]) == pytest.approx(0.03)
    c["model_id"] = ""
    with pytest.raises(ValueError):
        validate_scene(c)


def test_hermite_continuity_and_timing():
    knots = np.array([[0.0, 0.0], [0.1, 0.2], [0.3, 0.1]])
    t, q = hermite(knots, np.array([0.4, 0.4]), np.array([1.0, 1.0]), 0.01)
    v = np.diff(q, axis=0) / np.diff(t)[:, None]
    assert abs(v).max() <= 0.401
    assert abs(np.diff(v, axis=0) / 0.01).max() <= 1.01
    np.testing.assert_allclose(q[[0, -1]], knots[[0, -1]])
    assert q[:, 0].min() >= 0 and q[:, 0].max() <= 0.3 + 1e-8


def test_filter_deduplicates_original_acquisition_and_wraps_yaw():
    f = PoseFilter()
    a, new = f.update("x", 1.0, 1, [0, 0, 0, 3.13])
    assert new
    b, new = f.update("x", 1.0, 1, [100, 100, 0, 0])
    assert not new
    np.testing.assert_equal(a, b)
    c, new = f.update("x", 1.1, 2, [0.01, 0, 0, -3.13])
    assert new
    assert abs(c[2] - 3.14) < 0.1


def test_scan_selects_reachable_label_geometry(tmp_path):
    c = fixture(tmp_path)
    r = RobotModel(c["robot"])
    q = np.zeros(28)
    fk = r.fk(q)
    right, left = fk[r.ee["right"]], fk[r.ee["left"]]
    c["left_ee_from_reader"] = np.eye(4).tolist()
    c["read_distance"] = 0.15
    label_from_reader = np.diag([1.0, -1.0, -1.0, 1.0])
    label_from_reader[2, 3] = 0.15
    label = np.linalg.inv(right) @ left @ np.linalg.inv(label_from_reader)
    unreachable = label.copy()
    unreachable[0, 3] += 10
    c["object_from_labels"] = [unreachable.tolist(), label.tolist()]
    data = Planner(c).plan("Scan", q, right)
    final = RobotModel(c["robot"]).fk(data["path"][-1])
    np.testing.assert_allclose(final[r.ee["left"]], left, atol=0.003)
    assert data["ik_calls"] >= 3


def test_configuration_loads_relative_urdf_and_retains_startup_snapshot(tmp_path):
    import yaml
    from deux_task_core.configuration import load_configuration

    c = fixture(tmp_path)
    c["robot"]["urdf_path"] = "synthetic.urdf"
    path = tmp_path / "installation.yaml"
    path.write_text(yaml.safe_dump(c))
    first, version = load_configuration(path)
    second, other_version = load_configuration(path)
    assert first == second and version == other_version
    assert first["robot"]["urdf_path"] == str(tmp_path / "synthetic.urdf")
    c["lift_height"] = 0.08
    path.write_text(yaml.safe_dump(c))
    updated, new_version = load_configuration(path)
    assert first["lift_height"] == 0.03
    assert updated["lift_height"] == 0.08 and new_version != version
    with pytest.raises(ValueError, match="config_file"):
        load_configuration("")
    path.write_text("[]")
    with pytest.raises(ValueError, match="mapping"):
        load_configuration(path)


def test_observed_barcode_alignment_holds_right_arm(tmp_path):
    from deux_task_core.synthetic import fixture
    from deux_task_core.planning import Planner
    from deux_task_core.model import JOINT_NAMES
    c=fixture(tmp_path);c['left_ee_from_reader']=np.eye(4).tolist();c['read_distance']=.02
    planner=Planner(c);q=np.zeros(28)
    reader=np.diag([1.,-1.,-1.,1.]);reader[2,3]=.02
    label=planner.robot.fk(q)[planner.robot.ee['left']]@np.linalg.inv(reader)
    result=planner.plan('AlignBarcode',q,np.array(c['scan_right_pose']),barcode_pose=label)
    indices=[i for i,n in enumerate(JOINT_NAMES) if n.startswith('arm_r_')]
    np.testing.assert_allclose(result['path'][:,indices],0.,atol=1e-8)
    with pytest.raises(ValueError,match='surface required'):
        planner.plan('AlignBarcode',q,np.array(c['scan_right_pose']))


def test_joint_limit_numerical_tolerance_is_bounded(tmp_path):
    c=fixture(tmp_path);c['robot']['joint_limit_tolerance']=1e-4
    model=RobotModel(c['robot']);q=np.zeros(28);q[0]=model.limits[0,0]-1e-5
    model.clearance(q,[],0.)
    q[0]=model.limits[0,0]-.002
    with pytest.raises(ValueError,match='joint limit'):model.clearance(q,[],0.)
    c['robot']['joint_limit_tolerance']=.01
    with pytest.raises(ValueError,match='numerical tolerance'):RobotModel(c['robot'])
    c['simulation_wall_timeout_scale']=21
    with pytest.raises(ValueError,match='timeout scale'):validate_scene(c)


def test_loaded_motion_retains_closing_setpoint(tmp_path):
    from deux_task_core.model import HAND_NAMES,JOINT_NAMES
    c=fixture(tmp_path);planner=Planner(c)
    # Align at the current left pose to isolate loaded finger targets.
    c['left_ee_from_reader']=np.eye(4).tolist();c['read_distance']=.02
    q=np.zeros(28);label=planner.robot.fk(q)[planner.robot.ee['left']].copy()
    offset=np.diag([1.,-1.,-1.,1.]);offset[2,3]=.02
    result=planner.plan('AlignBarcode',q,np.array(c['scan_right_pose']),barcode_pose=label@np.linalg.inv(offset))
    indices=[JOINT_NAMES.index(n) for n in HAND_NAMES]
    np.testing.assert_allclose(result['path'][0,indices],c['hand_closed'])
    np.testing.assert_allclose(result['path'][-1,indices],c['hand_closed'])


def test_prepare_uses_ready_pose_without_object_and_checks_collision(tmp_path):
    c = fixture(tmp_path)
    c['home_joints'] = [0.02] * 28
    c['prepare_joint_waypoints'] = [[-0.01] * 28]
    planner = Planner(c)
    data = planner.plan('Prepare', np.zeros(28), None)
    np.testing.assert_allclose(data['path'][0], np.zeros(28))
    np.testing.assert_allclose(data['path'][-1], c['home_joints'])
    assert np.min(data['path'][:, 0]) == pytest.approx(-0.01)
    center = planner.robot.fk(np.zeros(28))[c['robot']['ee_links']['left']][:3, 3]
    c['obstacles'] = [dict(min=(center-.1).tolist(), max=(center+.1).tolist())]
    with pytest.raises(ValueError, match='collision'):
        Planner(c).plan('Prepare', np.zeros(28), None)


def test_loaded_scan_speed_scaling_preserves_endpoint_and_limits(tmp_path):
    c=fixture(tmp_path)
    c.update(perception_policy='head_rgbd',stationary_source_confirmed=True,grasp_pose_max_age=15.,
             observe_right_joints=[0.]*7,scan_right_joints=[.3,0.,0.,-.2,0.,0.,0.])
    fast=Planner(c).plan('ScanPose',np.zeros(28),None)
    c['scan_motion_scale']=.35
    slow=Planner(c).plan('ScanPose',np.zeros(28),None)
    np.testing.assert_allclose(slow['path'][-1],fast['path'][-1])
    assert slow['times'][-1] > fast['times'][-1]*2
    assert np.max(np.abs(np.diff(slow['path'],axis=0)/np.diff(slow['times'])[:,None])) <= .35*np.max(Planner(c).robot.velocity)
    for value in [0.,1.01,float('nan')]:
        c['scan_motion_scale']=value
        with pytest.raises(ValueError,match='scan_motion_scale'):validate_scene(c)


@pytest.mark.parametrize('angles',[(0.,0.),(-45.,-45.)])
def test_reader_roll_keeps_observed_point_and_beam_direction(tmp_path,angles):
    from scipy.spatial.transform import Rotation
    c=fixture(tmp_path);c['left_ee_from_reader']=np.eye(4).tolist();c['read_distance']=.02
    c['barcode_reader_roll_candidates']=[np.pi];c['barcode_view_angles_deg']=[list(angles)]
    planner=Planner(c);q=np.zeros(28)
    pitch,skew=np.deg2rad(angles);ray=np.array([np.tan(skew),np.tan(pitch),1.]);ray/=np.linalg.norm(ray)
    x=np.array([1.,0.,0.]);x-=ray*np.dot(x,ray);x/=np.linalg.norm(x)
    basis=np.column_stack([x,np.cross(ray,x),ray])
    offset=np.eye(4);offset[:3,:3]=basis@Rotation.from_rotvec([0.,0.,np.pi]).as_matrix()@np.diag([1.,-1.,-1.]);offset[:3,3]=.02*ray
    label=planner.robot.fk(q)[planner.robot.ee['left']]@np.linalg.inv(offset)
    result=planner.plan('AlignBarcode',q,np.array(c['scan_right_pose']),barcode_pose=label)
    actual=planner.robot.fk(result['path'][-1])[planner.robot.ee['left']]
    np.testing.assert_allclose(actual[:3,3],label[:3,3]+label[:3,:3]@ray*.02,atol=.002)
    np.testing.assert_allclose(actual[:3,2],-label[:3,:3]@ray,atol=.02)
    np.testing.assert_allclose(result['path'][:,14:21],0.,atol=1e-8)
    for value in [[],[float('nan')],[4.],[[0.]]]:
        c['barcode_reader_roll_candidates']=value
        with pytest.raises(ValueError,match='roll_candidates'):validate_scene(c)


def test_fast_ik_chain_matches_full_urdf_both_hands(tmp_path):
    r=RobotModel(fixture(tmp_path)['robot']);rng=np.random.default_rng(901)
    for _ in range(20):
        q=rng.uniform(r.limits[:,0],r.limits[:,1])
        all_links=r.fk(q)
        for side in ('left','right'):
            np.testing.assert_allclose(r.ee_fk(side,q),all_links[r.ee[side]],atol=1e-12)


def test_lift_timing_preserves_endpoints_and_joint_dynamics():
    knots=np.linspace([0.,0.],[.22,-.15],14)
    limits=np.array([.4,.4]);acceleration=np.array([1.,1.])
    old,_=hermite(knots,limits,acceleration,.02)
    t,q=hermite(np.linspace(knots[0],knots[-1],5),limits,acceleration,.02,.06)
    assert t[-1]<old[-1]*.75
    velocity=np.diff(q,axis=0)/np.diff(t)[:,None]
    accel=np.diff(np.vstack([np.zeros(2),velocity,np.zeros(2)]),axis=0)/np.r_[np.diff(t),np.diff(t)[-1]][:,None]
    assert np.all(np.max(abs(velocity),axis=0)<=limits+1e-5)
    assert np.all(np.max(abs(accel),axis=0)<=acceleration+1e-5)
    np.testing.assert_allclose(q[[0,-1]],knots[[0,-1]])


@pytest.mark.parametrize('key,value', [('lift_cartesian_step',.2),('lift_minimum_segment_duration',0.),('lift_start_lead',.05),('lift_start_lead',float('nan'))])
def test_lift_settings_reject_invalid_timing(tmp_path,key,value):
    c=fixture(tmp_path);c[key]=value
    with pytest.raises(ValueError,match=key):validate_scene(c)
