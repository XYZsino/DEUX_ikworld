"""Publish the current measured-lift snapshot; never issue robot commands."""
from pathlib import Path
import json,time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from deux_lift_controller_interfaces.msg import LiftCommand

class CameraSnapshot(Node):
    def __init__(self):
        super().__init__("thor_calibrated_camera_tf")
        self.root=Path(__file__).resolve().parents[1]
        self.config=json.loads((self.root/"calibration/zed2i_extrinsics.thor.json").read_text())
        self.lockfile=self.root/".setup/camera-calibration-lift-invalid.json"
        self.invalid=self.lockfile.exists()
        self.started=time.monotonic();self.publishing=False
        self.broadcaster=TransformBroadcaster(self)
        self.status=self.create_publisher(String,"/task/camera_calibration_status",10)
        for topic in ("/lift/control","/lift_control"):
            self.create_subscription(LiftCommand,topic,self.lift,qos_profile_sensor_data)
        self.create_subscription(TFMessage,"/tf",self.existing,qos_profile_sensor_data)
        self.create_subscription(TFMessage,"/tf_static",self.existing,QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE))
        self.timer=self.create_timer(1/30,self.tick)
        self.report_timer=self.create_timer(1,self.report)
    def invalidate(self,reason):
        if not self.invalid:
            self.invalid=True
            self.lockfile.write_text(json.dumps(dict(reason=reason,time=time.time()),indent=2))
            self.get_logger().error(reason+"; camera TF stopped. Re-measure/re-confirm lift height before rearming.")
    def lift(self,msg):
        if msg.command!=LiftCommand.STOP:self.invalidate("Lift movement command received")
    def existing(self,msg):
        if self.publishing:return
        if any(t.child_frame_id==self.config["child_frame"] for t in msg.transforms):
            self.invalidate("Camera frame already has a TF publisher")
    def report(self):
        msg=String();msg.data=json.dumps(dict(valid=not self.invalid,height_from_floor_m=self.config["camera_height_from_floor_m"],mode=self.config["mode"]))
        self.status.publish(msg)
    def tick(self):
        if self.invalid or time.monotonic()-self.started<3:return
        self.publishing=True
        t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg()
        t.header.frame_id=self.config["parent_frame"];t.child_frame_id=self.config["child_frame"]
        t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=self.config["translation_m"]
        t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=self.config["quaternion_xyzw"]
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init();node=CameraSnapshot()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=="__main__":main()
