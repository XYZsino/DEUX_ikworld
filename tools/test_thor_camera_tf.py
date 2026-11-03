"""ROS test on an isolated domain: valid TF, then invalidation on lift movement."""
import os,time,tempfile
from pathlib import Path
assert os.environ["ROS_DOMAIN_ID"]=="219"
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from tf2_msgs.msg import TFMessage
from deux_lift_controller_interfaces.msg import LiftCommand
from rclpy.qos import qos_profile_sensor_data
from thor_camera_tf import CameraSnapshot
rclpy.init()
with tempfile.TemporaryDirectory() as tmp:
    n=CameraSnapshot();n.lockfile=Path(tmp)/"invalid.json";n.invalid=False
    probe=Node("calibration_guard_test")
    seen=[]
    sub=probe.create_subscription(TFMessage,"/tf",lambda msg:seen.extend(t for t in msg.transforms if t.child_frame_id==n.config["child_frame"]),qos_profile_sensor_data)
    pub=probe.create_publisher(LiftCommand,"/lift/control",10)
    executor=SingleThreadedExecutor();executor.add_node(n);executor.add_node(probe)
    end=time.monotonic()+6
    while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
    assert len(seen)>20,len(seen)
    assert abs(seen[-1].transform.translation.z-n.config["translation_m"][2])<1e-9
    cmd=LiftCommand();cmd.command=LiftCommand.UP;pub.publish(cmd)
    end=time.monotonic()+.5
    while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
    assert n.invalid and n.lockfile.exists()
    count=len(seen)
    end=time.monotonic()+.5
    while time.monotonic()<end:executor.spin_once(timeout_sec=.02)
    assert len(seen)==count,(count,len(seen))
    print("PASS: calibrated TF published; movement invalidates snapshot and stops new TF.")
    executor.shutdown();n.destroy_node();probe.destroy_node()
rclpy.shutdown()
