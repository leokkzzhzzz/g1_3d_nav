#!/usr/bin/env python3
"""TCP → ROS 2 bridge receiver. Runs on G1 host (native ROS 2 Humble).

Connects to the sender on localhost:7777, reads newline-delimited JSON,
reconstructs ROS 2 messages, and publishes them.

Topic re-mapping (ROS 1 → ROS 2):
  /Odometry_loc   → /lidar_odometry/pose_fixed  (jie_3d_nav expects this)
  /localization_3d           → same
  /localization_3d_confidence → same
  /map                       → /map (PointCloud2)
"""

import json
import socket
import time
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from std_msgs.msg import Float32, Header
from sensor_msgs.msg import PointCloud2, PointField

PORT = 7777
HOST = "127.0.0.1"
RECONNECT_DELAY = 3.0

# (ros1_topic, ros1_type, ros2_topic, ros2_type, ros2_class)
BRIDGE_MAP = [
    ("/Odometry_loc", "nav_msgs/Odometry",
     "/lidar_odometry/pose_fixed", "nav_msgs/msg/Odometry", Odometry),
    ("/localization_3d", "geometry_msgs/PoseStamped",
     "/localization_3d", "geometry_msgs/msg/PoseStamped", PoseStamped),
    ("/localization_3d_confidence", "std_msgs/Float32",
     "/localization_3d_confidence", "std_msgs/msg/Float32", Float32),
    ("/map", "sensor_msgs/PointCloud2",
     "/map", "sensor_msgs/msg/PointCloud2", PointCloud2),
]


def build_odometry(data):
    """Construct nav_msgs/Odometry from dict."""
    msg = Odometry()
    d = data.get("pose", {}).get("pose", {})
    pos = d.get("position", {})
    ori = d.get("orientation", {})
    msg.pose.pose.position = Point(
        x=float(pos.get("x", 0)),
        y=float(pos.get("y", 0)),
        z=float(pos.get("z", 0)),
    )
    msg.pose.pose.orientation = Quaternion(
        x=float(ori.get("x", 0)),
        y=float(ori.get("y", 0)),
        z=float(ori.get("z", 0)),
        w=float(ori.get("w", 1)),
    )
    # twist
    tw = data.get("twist", {}).get("twist", {})
    msg.twist.twist.linear.x = float(tw.get("linear", {}).get("x", 0))
    msg.twist.twist.linear.y = float(tw.get("linear", {}).get("y", 0))
    msg.twist.twist.linear.z = float(tw.get("linear", {}).get("z", 0))
    msg.twist.twist.angular.x = float(tw.get("angular", {}).get("x", 0))
    msg.twist.twist.angular.y = float(tw.get("angular", {}).get("y", 0))
    msg.twist.twist.angular.z = float(tw.get("angular", {}).get("z", 0))
    # header
    h = data.get("header", {})
    msg.header.frame_id = str(h.get("frame_id", "odom"))
    msg.child_frame_id = str(data.get("child_frame_id", "base_link"))
    return msg


def build_pose_stamped(data):
    """Construct geometry_msgs/PoseStamped from dict."""
    msg = PoseStamped()
    d = data.get("pose", {})
    msg.pose.position = Point(
        x=float(d.get("position", {}).get("x", 0)),
        y=float(d.get("position", {}).get("y", 0)),
        z=float(d.get("position", {}).get("z", 0)),
    )
    msg.pose.orientation = Quaternion(
        x=float(d.get("orientation", {}).get("x", 0)),
        y=float(d.get("orientation", {}).get("y", 0)),
        z=float(d.get("orientation", {}).get("z", 0)),
        w=float(d.get("orientation", {}).get("w", 1)),
    )
    h = data.get("header", {})
    msg.header.frame_id = str(h.get("frame_id", "map"))
    return msg


def build_float32(data):
    msg = Float32()
    msg.data = float(data.get("data", 0))
    return msg


def build_pointcloud2(data):
    """Construct sensor_msgs/PointCloud2 from dict. Handles both flat and nested reprs."""
    msg = PointCloud2()
    h = data.get("header", {})
    msg.header.frame_id = str(h.get("frame_id", "map"))
    msg.height = int(data.get("height", 1))
    msg.width = int(data.get("width", 0))
    msg.point_step = int(data.get("point_step", 0))
    msg.row_step = int(data.get("row_step", 0))
    msg.is_dense = bool(data.get("is_dense", True))

    fields = data.get("fields", [])
    for f in fields:
        pf = PointField()
        pf.name = str(f.get("name", ""))
        pf.offset = int(f.get("offset", 0))
        pf.datatype = int(f.get("datatype", 0))
        pf.count = int(f.get("count", 0))
        msg.fields.append(pf)

    raw = data.get("data", [])
    if isinstance(raw, list):
        msg.data = bytes(bytearray(raw))
    elif isinstance(raw, str):
        # base64 or escaped string repr
        try:
            msg.data = bytes(bytearray(json.loads(raw)))
        except Exception:
            msg.data = b""
    else:
        msg.data = b""

    return msg


BUILDERS = {
    "nav_msgs/Odometry": build_odometry,
    "geometry_msgs/PoseStamped": build_pose_stamped,
    "std_msgs/Float32": build_float32,
    "sensor_msgs/PointCloud2": build_pointcloud2,
}


class BridgeReceiver(Node):
    def __init__(self):
        super().__init__("bridge_receiver")
        self.pubs = {}
        for r1_topic, r1_type, r2_topic, r2_type, r2_cls in BRIDGE_MAP:
            self.pubs[r1_type] = self.create_publisher(r2_cls, r2_topic, 10)
            self.get_logger().info(f"  pub ROS2 {r2_topic} ({r2_type})")
        self.sock = None

    def connect(self):
        while rclpy.ok():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(1.0)
                self.sock.connect((HOST, PORT))
                self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info(f"Connected to bridge sender at {HOST}:{PORT}")
                return
            except Exception:
                self.get_logger().warn(
                    f"Waiting for bridge sender... retry in {RECONNECT_DELAY}s"
                )
                time.sleep(RECONNECT_DELAY)

    def run(self):
        buf = b""
        while rclpy.ok():
            if self.sock is None:
                self.connect()
                buf = b""
                continue

            try:
                chunk = self.sock.recv(65536)
                if not chunk:
                    raise ConnectionError("disconnected")
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self._handle(line)
            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f"Connection lost: {e}")
                try:
                    self.sock.close()
                except Exception:
                    pass
                self.sock = None

    def _handle(self, line):
        try:
            data = json.loads(line.decode())
            ros1_type = data["type"]
            msg_data = data["msg"]

            builder = BUILDERS.get(ros1_type)
            if builder is None:
                self.get_logger().warn(f"No builder for type: {ros1_type}")
                return

            msg = builder(msg_data)
            pub = self.pubs.get(ros1_type)
            if pub:
                pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Parse error: {e}")


def main():
    rclpy.init(args=sys.argv)
    node = BridgeReceiver()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
