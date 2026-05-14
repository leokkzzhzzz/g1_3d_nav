#!/usr/bin/env python3
"""G1 base controller — pure pursuit path tracking via Unitree SDK2.

Subscribes:
  /lidar_odometry/pose_fixed  (nav_msgs/Odometry)  — robot pose
  /planned_path               (nav_msgs/Path)       — planned path from jie_3d_nav
  /start_navigation           (std_msgs/Bool)       — start command
  /stop_navigation            (std_msgs/Bool)       — stop command

Publishes:
  /cmd_vel                    (geometry_msgs/Twist) — for monitoring

Drives the G1 base via Unitree SDK2 LocoClient.Move(vx, vy, vyaw).

Requires: unitree_sdk2py, G1_INTERFACE env var
"""

import math
import os
import sys
import time
from threading import Lock

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

if os.environ.get("G1_USE_REAL_ROBOT", "1") != "0":
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize


def yaw_from_quat(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class G1BaseController(Node):
    def __init__(self):
        super().__init__("g1_base_controller")

        # Parameters (mirror d1_controller defaults)
        self.declare_parameter("lookahead_distance", 0.3)
        self.declare_parameter("linear_gain", 1.0)
        self.declare_parameter("lateral_gain", 0.8)
        self.declare_parameter("heading_gain", 0.8)
        self.declare_parameter("max_linear_speed", 0.6)
        self.declare_parameter("max_lateral_speed", 0.0)  # G1 walks forward
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("linear_deadband", 0.05)
        self.declare_parameter("angular_deadband", 0.05)
        self.declare_parameter("goal_position_tolerance", 0.15)
        self.declare_parameter("goal_yaw_tolerance", 0.2)
        self.declare_parameter("tracking_point_reached_xy_tolerance", 0.15)
        self.declare_parameter("control_frequency", 20.0)
        self.declare_parameter("interface", os.environ.get("G1_INTERFACE", "eth0"))

        # State
        self.path = []          # [(x, y, yaw), ...]
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.have_pose = False
        self.nav_active = False
        self.target_idx = 0
        self.goal_reached = True
        self.lock = Lock()

        # Unitree SDK2
        self.robot = None
        interface = self.get_parameter("interface").value
        if os.environ.get("G1_USE_REAL_ROBOT", "1") != "0":
            ChannelFactoryInitialize(0, interface)
            self.robot = LocoClient()
            self.robot.SetTimeout(10.0)
            self.robot.SetFsmId(4)
            self.robot.Init()
            self.robot.Damp()
            self.get_logger().info(f"G1 SDK2 initialized on {interface}")
        else:
            self.get_logger().info("G1_USE_REAL_ROBOT=0, simulation mode")

        # Subscribers
        self.create_subscription(Odometry, "/lidar_odometry/pose_fixed", self.cb_odom, 10)
        self.create_subscription(Path, "/planned_path", self.cb_path, 10)
        self.create_subscription(Bool, "/start_navigation", self.cb_start, 10)
        self.create_subscription(Bool, "/stop_navigation", self.cb_stop, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Control timer
        freq = self.get_parameter("control_frequency").value
        self.timer = self.create_timer(1.0 / freq, self.control_loop)

        self.get_logger().info("G1 base controller ready")

    # ── callbacks ──────────────────────────────────────────

    def cb_odom(self, msg: Odometry):
        with self.lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_yaw = yaw_from_quat(
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w,
            )
            self.have_pose = True

    def cb_path(self, msg: Path):
        with self.lock:
            self.path = []
            for p in msg.poses:
                pt = p.pose.position
                yaw = yaw_from_quat(
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                )
                self.path.append((pt.x, pt.y, yaw))
            self.target_idx = 0
            self.goal_reached = False
            self.get_logger().info(f"Received path: {len(self.path)} waypoints")

    def cb_start(self, msg: Bool):
        if msg.data and self.path:
            with self.lock:
                self.nav_active = True
                self.goal_reached = False
                self.target_idx = self._nearest_index()
            self.get_logger().info("Navigation STARTED")

    def cb_stop(self, msg: Bool):
        if msg.data:
            with self.lock:
                self.nav_active = False
            self._stop_robot()
            self.get_logger().info("Navigation STOPPED")

    # ── pure pursuit core ──────────────────────────────────

    def _nearest_index(self):
        best, bd = 0, float("inf")
        for i, (px, py, _) in enumerate(self.path):
            d = (px - self.robot_x) ** 2 + (py - self.robot_y) ** 2
            if d < bd:
                bd, best = d, i
        return best

    def _lookahead_point(self, idx):
        L = self.get_parameter("lookahead_distance").value
        remaining = L
        prev = (self.robot_x, self.robot_y)
        j = idx
        while j < len(self.path) - 1:
            px, py, _ = self.path[j + 1]
            seg = math.hypot(px - prev[0], py - prev[1])
            if seg >= remaining:
                frac = remaining / max(seg, 1e-9)
                return (prev[0] + frac * (px - prev[0]), prev[1] + frac * (py - prev[1]))
            remaining -= seg
            prev = (self.path[j + 1][0], self.path[j + 1][1])
            j += 1
        return (self.path[-1][0], self.path[-1][1])

    def _is_final_point(self):
        return self.target_idx >= len(self.path) - 1

    def control_loop(self):
        with self.lock:
            if not self.nav_active or not self.path or not self.have_pose:
                return

            # Skip reached waypoints
            tol = self.get_parameter("tracking_point_reached_xy_tolerance").value
            while self.target_idx < len(self.path) - 1:
                px, py, _ = self.path[self.target_idx]
                if math.hypot(px - self.robot_x, py - self.robot_y) < tol:
                    self.target_idx += 1
                else:
                    break

            # Goal check
            if self._is_final_point():
                gx, gy, gyaw = self.path[-1]
                dist = math.hypot(gx - self.robot_x, gy - self.robot_y)
                if dist < self.get_parameter("goal_position_tolerance").value:
                    self.get_logger().info("Goal reached!")
                    self.nav_active = False
                    self.goal_reached = True
                    self._stop_robot()
                    return

            # Compute lookahead target
            tx, ty = self._lookahead_point(self.target_idx)
            dx = tx - self.robot_x
            dy = ty - self.robot_y
            target_in_base_x = dx * math.cos(-self.robot_yaw) - dy * math.sin(-self.robot_yaw)
            target_in_base_y = dx * math.sin(-self.robot_yaw) + dy * math.cos(-self.robot_yaw)

            # Velocity from errors
            lg = self.get_parameter("linear_gain").value
            hg = self.get_parameter("heading_gain").value
            heading_error = math.atan2(target_in_base_y, max(target_in_base_x, 1e-6))

            vx = clamp(target_in_base_x * lg,
                       -self.get_parameter("max_linear_speed").value,
                       self.get_parameter("max_linear_speed").value)
            vy = clamp(target_in_base_y * self.get_parameter("lateral_gain").value,
                       -self.get_parameter("max_lateral_speed").value,
                       self.get_parameter("max_lateral_speed").value)
            wz = clamp(heading_error * hg,
                       -self.get_parameter("max_angular_speed").value,
                       self.get_parameter("max_angular_speed").value)

            # Deadband
            l_db = self.get_parameter("linear_deadband").value
            a_db = self.get_parameter("angular_deadband").value
            if abs(vx) < l_db:
                vx = 0.0
            if abs(vy) < l_db:
                vy = 0.0
            if abs(wz) < a_db:
                wz = 0.0

            # Publish and drive
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz
            self.cmd_vel_pub.publish(twist)

            if self.robot is not None:
                if abs(vx) < 0.03 and abs(vy) < 0.03 and abs(wz) < 0.03:
                    self.robot.StopMove()
                else:
                    self.robot.Move(vx=vx, vy=vy, vyaw=wz, continous_move=True)

    def _stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        if self.robot is not None:
            self.robot.StopMove()
            self.robot.Damp()

    def destroy_node(self):
        self._stop_robot()
        super().destroy_node()


def main():
    rclpy.init(args=sys.argv)
    node = G1BaseController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
