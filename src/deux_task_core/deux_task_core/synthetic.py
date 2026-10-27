"""Explicitly synthetic fixture builder. Never a real DEUX installation config."""

from pathlib import Path
import numpy as np
from .model import JOINT_NAMES, RobotModel


def fixture(directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    xml = ['<robot name="synthetic"><link name="base_link"/>']
    for side in ("left", "right"):
        parent = "base_link"
        names = [
            n
            for n in JOINT_NAMES
            if n.startswith("arm_" + side[0] + "_") or n.startswith(side + "_hand")
        ]
        for i, name in enumerate(names):
            link = name + "_link"
            xml.append(f'<link name="{link}"/>')
            xyz = (
                f'0 {0.4 if side=="left" else -0.4} 0'
                if i == 0
                else ("0.08 0 0" if i < 7 else "0.005 0 0")
            )
            axis = ("0 0 1", "0 1 0", "1 0 0")[i % 3]
            xml.append(
                f'<joint name="{name}" type="revolute"><parent link="{parent}"/><child link="{link}"/><origin xyz="{xyz}"/><axis xyz="{axis}"/><limit lower="-2.5" upper="2.5" velocity="1.5" effort="10"/></joint>'
            )
            parent = link
    xml.append("</robot>")
    path = directory / "synthetic.urdf"
    path.write_text("".join(xml))
    robot = dict(
        urdf_path=str(path),
        base_frame="base_link",
        ee_links={s: f"arm_{s[0]}_joint7_link" for s in ("left", "right")},
        acceleration_limits=[4.0] * 28,
        collision_spheres=[
            dict(link=f"arm_{s}_joint7_link", center=[0, 0, 0], radius=0.02)
            for s in ("l", "r")
        ],
        attachment_touch_links=["arm_r_joint7_link"],
    )
    model = RobotModel(robot)
    q = np.zeros(28)
    links = model.fk(q)
    left = links[robot["ee_links"]["left"]].tolist()
    right = links[robot["ee_links"]["right"]].tolist()
    return dict(
        schema="deux.scene.v1",
        synthetic=True,
        ik_backend="scipy",  # Lightweight synthetic tests explicitly select their engine.
        base_frame="base_link",
        model_id="synthetic-object",
        robot=robot,
        source_box_height=0.07,
        grasp_object_from_right_ee=np.eye(4).tolist(),
        scan_right_pose=right,
        scan_left_pose=left,
        retreat_left_pose=left,
        destination_pose=right,
        hand_open=[0.0] * 14,
        hand_closed=[0.03] * 14,
        home_joints=q.tolist(),
        workspace=dict(min=[-2, -2, -2], max=[2, 2, 2]),
        destination_bounds=dict(min=[0.3, -0.6, -0.2], max=[0.6, -0.2, 0.2]),
        obstacles=[],
        safe_joint_waypoint=None,
        approach_height=0.02,
        lift_height=0.03,
        attached_radius=0.05,
        max_target_drift=0.02,
        max_observation_age=1.0,
        max_joint_age=0.5,
        max_object_age=1.0,
        max_plan_age=2.0,
        sample_period=0.02,
        collision_margin=0.01,
        start_tolerance=0.05,
        tracking_tolerance=0.2,
        settle_timeout=1.0,
        scan_timeout=0.3,
        verification_timeout=0.5,
    )
