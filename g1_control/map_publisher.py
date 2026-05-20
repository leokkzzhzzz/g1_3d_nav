#!/usr/bin/env python3
"""Publish /map point cloud persistently (like ROS1 latch=true).
Reads the PCD file and republishes every 5s so late-joining RViz2 always sees it."""

import struct, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

PCD_PATH = "/root/maps/scans.pcd"
PUBLISH_HZ = 0.2  # republish every 5 seconds


class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_publisher")

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(PointCloud2, "/map", qos)

        # Read PCD
        self._pc2 = self._read_pcd(PCD_PATH)
        if self._pc2 is None:
            self.get_logger().error(f"Failed to read {PCD_PATH}")
            return

        self.get_logger().info(f"Loaded {len(self._pc2.data)} bytes, {self._pc2.width} points")
        self._pub.publish(self._pc2)
        self.get_logger().info("/map published (TRANSIENT_LOCAL)")
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._republish)

    def _republish(self):
        self._pub.publish(self._pc2)

    def _read_pcd(self, path):
        try:
            with open(path, "rb") as f:
                header = b""
                while True:
                    line = f.readline()
                    header += line
                    if line.startswith(b"DATA"):
                        break
                raw = f.read()

            hdr = header.decode()
            fields = [l.split()[1] for l in hdr.split("\n") if l.startswith("FIELDS")]
            sizes = [l.split()[1] for l in hdr.split("\n") if l.startswith("SIZE")]
            types = [l.split()[1] for l in hdr.split("\n") if l.startswith("TYPE")]
            npoints = int([l.split()[1] for l in hdr.split("\n") if l.startswith("POINTS")][0])

            msg = PointCloud2()
            msg.header.frame_id = "map"
            msg.height = 1
            msg.width = npoints
            msg.is_dense = True
            msg.point_step = sum(int(s) for s in sizes)
            msg.row_step = msg.point_step * npoints

            type_map = {"F": PointField.FLOAT32, "I": PointField.INT32, "U": PointField.UINT32}
            msg.fields = []
            offset = 0
            for fname, fsize, ftype in zip(fields, sizes, types):
                msg.fields.append(PointField(
                    name=fname, offset=offset,
                    datatype=type_map.get(ftype, PointField.FLOAT32),
                    count=1))
                offset += int(fsize)

            msg.data = raw
            return msg
        except Exception as e:
            self.get_logger().error(str(e))
            return None


def main():
    rclpy.init()
    node = MapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
