"""Run only in an isolated ROS domain; synthetic inputs, no hardware drivers."""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from deux_lift_controller_interfaces.msg import LiftCommand


def test_synthetic_outputs_and_sigterm_stop():
    assert os.environ.get('ROS_DOMAIN_ID')=='219'
    rclpy.init();node=Node('joycon_test_probe')
    received=[];lifts=[]
    node.create_subscription(Twist,'/mobile/cmd_vel',lambda m:received.append((m.linear.x,m.linear.y,m.angular.z)),10)
    node.create_subscription(LiftCommand,'/lift/control',lambda m:lifts.append(m.command),10)
    pubs={s:node.create_publisher(Joy,'/ikworld/joy/'+s,10) for s in ('left','right')}
    def pump(seconds,button=False,moving=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            for s,p in pubs.items():
                a=[0.]*6;b=[0]*11
                a[4 if s=='left' else 5]=1.
                b[4 if s=='left' else 6]=int(button)
                if moving:a[1 if s=='left' else 3]=1.
                p.publish(Joy(axes=a,buttons=b))
            rclpy.spin_once(node,timeout_sec=.02);time.sleep(.01)
    script=Path(__file__).resolve().parents[1]/'tools/joycon_mobile_lift_node.py'
    proc=subprocess.Popen([sys.executable,str(script)],stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
    try:
        pump(1.5);assert proc.poll() is None
        endpoints=dict(node.get_publisher_names_and_types_by_node('ikworld_joycon_mobile_lift','/'))
        assert '/mobile/cmd_vel' in endpoints and '/lift/control' in endpoints
        assert not any(t.startswith(('/hand/','/arm/','/deux/teleop/')) for t in endpoints)
        pump(.2,button=True);pump(.3,moving=True)
        assert any(x[0]>.39 for x in received) and 1 in lifts
        received.clear();lifts.clear()
        # Input silence must stop both channels.
        end=time.monotonic()+.8
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        assert received[-1]==(0.,0.,0.) and lifts[-1]==0
        pump(.1);pump(.1,button=True);pump(.2,moving=True)
        assert lifts[-1]==1
        proc.send_signal(signal.SIGTERM)
        end=time.monotonic()+1.
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        assert proc.wait(timeout=3)==0
        assert received[-1]==(0.,0.,0.) and lifts[-1]==0
    finally:
        if proc.poll() is None:proc.terminate();proc.wait(timeout=3)
        node.destroy_node();rclpy.shutdown()


def test_check_only_has_no_command_publishers():
    assert os.environ.get('ROS_DOMAIN_ID')=='219'
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
    from joycon_mobile_lift_node import MobileLift
    rclpy.init(args=['--ros-args','-p','check_only:=true'])
    node=MobileLift()
    try:
        assert node.cmd is None and node.lift is None
    finally:node.destroy_node();rclpy.shutdown()
