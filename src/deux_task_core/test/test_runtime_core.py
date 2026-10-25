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
