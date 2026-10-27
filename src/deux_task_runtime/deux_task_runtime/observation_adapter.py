"""Strict opt-in bridge to legacy sim JSON; never silently drops time/axis limits."""

from .common import joint_array, pose_matrix, seconds
from deux_task_core.model import JOINT_NAMES
from geometry_msgs.msg import Pose


def transform_pose(transform):
    p = Pose()
    p.position.x = transform.translation.x
    p.position.y = transform.translation.y
    p.position.z = transform.translation.z
    p.orientation = transform.rotation
    return pose_matrix(p).tolist()


def legacy_packet(observation, barcode_reads=(), *, joints=None):
    """v1 lacks per-object timestamps/axis flags: require exact-time complete poses.

    Delayed yaw-only ICP must remain in the ROS contract, not be passed to v1.
    The runtime nodes use the richer ROS messages and do not call this adapter.
    """
    if joints is None:
        raise ValueError("legacy adapter requires explicit measured joints")
    o = observation
    if not (
        o.joints_valid and o.left_ee_valid and o.right_ee_valid and o.objects_valid
    ):
        raise ValueError("legacy adapter requires measured joints, both EE and objects")
    if o.objects.header.frame_id != "base_link":
        raise ValueError("legacy schema requires base_link")
    if o.objects.header.stamp != o.header.stamp:
        raise ValueError("legacy schema cannot preserve delayed pose acquisition")
    if any(not d.full_orientation or d.pose_filtered for d in o.objects.detections):
        raise ValueError("legacy schema cannot express yaw-only or upstream filtering")
    for ee in (o.base_from_left_ee, o.base_from_right_ee):
        if ee.header.frame_id != "base_link" or ee.header.stamp != o.header.stamp:
            raise ValueError("legacy EE frame/time mismatch")
    if joints.header.stamp != o.header.stamp:
        raise ValueError("legacy joint timestamp mismatch")
    readings = []
    for read in barcode_reads:
        if seconds(read.header.stamp) > seconds(o.header.stamp):
            raise ValueError("future barcode")
        readings.append(
            dict(
                timestamp=seconds(read.header.stamp),
                value=read.value,
                track_id=read.track_id,
                simulated=read.simulated,
            )
        )
    return dict(
        schema="deux.observation.v1",
        frame="base_link",
        length_unit="m",
        angle_unit="rad",
        timestamp=seconds(o.header.stamp),
        sequence=o.sequence,
        joints=dict(names=list(JOINT_NAMES), positions=joint_array(joints).tolist()),
        ee=dict(
            left=transform_pose(o.base_from_left_ee.transform),
            right=transform_pose(o.base_from_right_ee.transform),
        ),
        cameras={},
        objects=[
            dict(
                track_id=d.track_id,
                pose=pose_matrix(d.pose.pose).tolist(),
                camera="external",
                confidence=d.confidence,
            )
            for d in o.objects.detections
        ],
        barcode_reads=readings,
        labels=[],
    )
