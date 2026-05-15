#!/usr/bin/env python3
"""Minimal bridge: Twist → Unitree SDK2. Runs on G1 host (Python 3.10 + ROS 2)."""

import os, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

G1_INTERFACE = os.environ.get("G1_INTERFACE", "eth0")

from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize


class CmdVelToSDK2(Node):
    def __init__(self):
        super().__init__("cmd_vel_to_sdk2")
        self.declare_parameter("interface", G1_INTERFACE)
        iface = self.get_parameter("interface").value

        ChannelFactoryInitialize(0, iface)
        self.robot = LocoClient()
        self.robot.SetTimeout(10.0)
        self.robot.SetFsmId(4)
        self.robot.Init()
        self.robot.Damp()
        self.get_logger().info(f"G1 SDK2 initialized on {iface}")

        self.create_subscription(Twist, "/cmd_vel_smooth", self.cb, 10)

    def cb(self, msg: Twist):
        vx, vy, wz = msg.linear.x, msg.linear.y, msg.angular.z
        if abs(vx) < 0.03 and abs(vy) < 0.03 and abs(wz) < 0.03:
            self.robot.StopMove()
        else:
            self.robot.Move(vx=vx, vy=vy, vyaw=wz, continous_move=True)

    def destroy_node(self):
        self.robot.StopMove()
        self.robot.Damp()
        super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    node = CmdVelToSDK2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
