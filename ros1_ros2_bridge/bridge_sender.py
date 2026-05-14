#!/usr/bin/env python3
"""ROS 1 → TCP bridge sender. Runs inside hongtu_mapper container.

Subscribes to configured ROS 1 topics and forwards each message as a
newline-delimited JSON line to every connected TCP client on port 7777.
"""

import json
import socket
import struct
import threading

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from sensor_msgs.msg import PointCloud2

PORT = 7777
HOST = "127.0.0.1"

clients = []
clients_lock = threading.Lock()


def _pose_dict(pose):
    return {
        "position": {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z},
        "orientation": {
            "x": pose.orientation.x, "y": pose.orientation.y,
            "z": pose.orientation.z, "w": pose.orientation.w,
        },
    }


def _header_dict(h):
    return {"frame_id": h.frame_id, "stamp": {"secs": h.stamp.secs, "nsecs": h.stamp.nsecs}}


def serialize_odometry(msg):
    return {
        "header": _header_dict(msg.header),
        "child_frame_id": msg.child_frame_id,
        "pose": {"pose": _pose_dict(msg.pose.pose)},
        "twist": {
            "twist": {
                "linear": {"x": msg.twist.twist.linear.x, "y": msg.twist.twist.linear.y, "z": msg.twist.twist.linear.z},
                "angular": {"x": msg.twist.twist.angular.x, "y": msg.twist.twist.angular.y, "z": msg.twist.twist.angular.z},
            }
        },
    }


def serialize_pose_stamped(msg):
    return {
        "header": _header_dict(msg.header),
        "pose": _pose_dict(msg.pose),
    }


def serialize_float32(msg):
    return {"data": msg.data}


def serialize_pointcloud2(msg):
    fields = []
    for f in msg.fields:
        fields.append({"name": f.name, "offset": f.offset, "datatype": f.datatype, "count": f.count})
    # Encode binary data as list of ints (safe across JSON)
    data_ints = list(bytearray(msg.data))
    return {
        "header": _header_dict(msg.header),
        "height": msg.height,
        "width": msg.width,
        "fields": fields,
        "is_bigendian": msg.is_bigendian,
        "point_step": msg.point_step,
        "row_step": msg.row_step,
        "is_dense": msg.is_dense,
        "data": data_ints,
    }


SERIALIZERS = {
    "nav_msgs/Odometry": serialize_odometry,
    "geometry_msgs/PoseStamped": serialize_pose_stamped,
    "std_msgs/Float32": serialize_float32,
    "sensor_msgs/PointCloud2": serialize_pointcloud2,
}

# (ros_topic, ros_type, ros_class)
TOPICS = [
    ("/Odometry_loc", "nav_msgs/Odometry", Odometry),
    ("/localization_3d", "geometry_msgs/PoseStamped", PoseStamped),
    ("/localization_3d_confidence", "std_msgs/Float32", Float32),
    ("/map", "sensor_msgs/PointCloud2", PointCloud2),
]


def make_callback(topic, msg_type):
    serializer = SERIALIZERS[msg_type]

    def cb(msg):
        line = json.dumps({
            "topic": topic,
            "type": msg_type,
            "msg": serializer(msg),
        }) + "\n"

        with clients_lock:
            dead = []
            for c in clients:
                try:
                    c.sendall(line.encode())
                except Exception:
                    dead.append(c)
            for c in dead:
                clients.remove(c)
    return cb


def accept_loop(server):
    while not rospy.is_shutdown():
        try:
            conn, addr = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with clients_lock:
                clients.append(conn)
            rospy.loginfo(f"Bridge client connected: {addr}")
        except socket.timeout:
            continue
        except Exception as e:
            rospy.logerr(f"Accept error: {e}")
            break


def main():
    rospy.init_node("bridge_sender", anonymous=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(1.0)
    server.bind((HOST, PORT))
    server.listen(5)
    rospy.loginfo(f"Bridge sender listening on {HOST}:{PORT}")

    t = threading.Thread(target=accept_loop, args=(server,), daemon=True)
    t.start()

    for topic, msg_type, msg_cls in TOPICS:
        rospy.Subscriber(topic, msg_cls, make_callback(topic, msg_type))
        rospy.loginfo(f"  sub ROS1 {topic} ({msg_type})")

    rospy.loginfo("Bridge sender running. Waiting for receiver...")
    rospy.spin()
    server.close()


if __name__ == "__main__":
    main()
