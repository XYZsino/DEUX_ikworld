#!/usr/bin/env python3
"""Standalone Joy-Con mobile/lift node. Publishes no arm/hand commands."""
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from deux_lift_controller_interfaces.msg import LiftCommand
from joycon_mobile_lift_logic import Control


class MobileLift(Node):
    def __init__(self):
        super().__init__('ikworld_joycon_mobile_lift')
        self.control=Control()
        self.stopping=False
        self.check=bool(self.declare_parameter('check_only',False).value)
        self.cmd=None if self.check else self.create_publisher(Twist,'/mobile/cmd_vel',10)
        self.lift=None if self.check else self.create_publisher(LiftCommand,'/lift/control',10)
        for side in ('left','right'):
            self.create_subscription(Joy,'/ikworld/joy/'+side,
                                     lambda m,s=side:self.joy(s,m),10)
        self.create_subscription(Bool,'/deux/estop',lambda m:self.control.emergency(m.data),10)
        self.create_timer(.05,self.tick)
        self.last_state=None
        self.last_check=0.
        self.last_lift=None
        self.last_lift_time=0.
        self.get_logger().info('Joy-Con mobile/lift only; + translation/rotation, - lift. '
                               +('CHECK ONLY: no command publishers.' if self.check else 'Disabled until toggled; neutral commands sent while disabled.'))

    def joy(self,side,msg):
        if self.check:
            now=time.monotonic()
            if now-self.last_check>.5:
                self.get_logger().info(f'{side}: axes={list(msg.axes)} buttons={list(msg.buttons)}')
                self.last_check=now
        if not self.stopping:
            self.control.feed(side,msg.axes,msg.buttons,time.monotonic())

    def send(self,velocity,lift,force=False):
        if self.check:return
        msg=Twist();msg.linear.x,msg.linear.y,msg.angular.z=velocity
        self.cmd.publish(msg)
        now=time.monotonic()
        if force or lift!=self.last_lift or now-self.last_lift_time>=.1:
            self.lift.publish(LiftCommand(command=lift))
            self.last_lift,self.last_lift_time=lift,now

    def tick(self):
        velocity,lift=self.control.output(time.monotonic())
        self.send(velocity,lift)
        state=tuple(self.control.enabled.values())
        if state!=self.last_state:
            self.get_logger().info(f'lift={state[0]}, translation/rotation={state[1]}')
            self.last_state=state


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node=MobileLift()
    def stop(*_):
        node.stopping=True
        node.control.stop()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stop)
    try:
        while rclpy.ok() and not node.stopping:rclpy.spin_once(node,timeout_sec=.05)
    finally:
        node.stopping=True;node.control.stop()
        # Deliver repeated STOP on normal shutdown/signals while DDS is alive.
        # Delivery is not a hardware acknowledgement; SIGKILL cannot run this.
        until=time.monotonic()+.5
        while rclpy.ok() and time.monotonic()<until:
            node.send((0.,0.,0.),0,force=True)
            rclpy.spin_once(node,timeout_sec=.05)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
