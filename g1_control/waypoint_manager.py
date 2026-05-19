#!/usr/bin/env python3
"""G1 Waypoint Manager — save / list / delete / navigate to waypoints.

Storage: /root/maps/waypoints.yaml (persistent across reboots)

ROS2 Humble Trigger has NO request fields, so waypoints are auto-named:
  /save_waypoint     → saves as "wp_1", "wp_2", ...
  /navigate_last     → navigates to most recently saved waypoint
  /delete_last       → deletes most recently saved waypoint
  /list_waypoints    → lists all saved waypoints
"""

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from nav2_msgs.action import NavigateToPose

WAYPOINTS_PATH = "/root/maps/waypoints.yaml"


def _load():
    try:
        with open(WAYPOINTS_PATH) as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}


def _save(data: dict):
    with open(WAYPOINTS_PATH, "w") as f:
        yaml.safe_dump(data, f, default_flow_style=False, allow_unicode=True)


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")

        self._wp = _load()
        self._last_name = max(self._wp.keys()) if self._wp else None
        self._counter = len(self._wp) + 1
        self.get_logger().info(f"Loaded {len(self._wp)} waypoints, last={self._last_name}")

        # Current pose from localization
        self._latest_pose = None
        self.create_subscription(PoseStamped, "/localization_3d", self._on_pose, 10)

        # Services
        self.create_service(Trigger, "/save_waypoint", self._handle_save)
        self.create_service(Trigger, "/delete_last", self._handle_delete)
        self.create_service(Trigger, "/list_waypoints", self._handle_list)
        self.create_service(Trigger, "/navigate_last", self._handle_nav)

        # Nav2 action client
        self._nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.get_logger().info("Ready: /save_waypoint /delete_last /list_waypoints /navigate_last")

    def _on_pose(self, msg: PoseStamped):
        self._latest_pose = msg

    # ── save ────────────────────────────────────────────

    def _handle_save(self, req, resp):
        if self._latest_pose is None:
            resp.success = False
            resp.message = "No localization yet. Wait for open3d_loc."
            return resp

        name = f"wp_{self._counter}"
        self._counter += 1
        p = self._latest_pose.pose.position
        o = self._latest_pose.pose.orientation
        wp = {
            "position": {"x": round(p.x, 4), "y": round(p.y, 4), "z": round(p.z, 4)},
            "orientation": {"x": round(o.x, 6), "y": round(o.y, 6),
                            "z": round(o.z, 6), "w": round(o.w, 6)},
        }
        self._wp[name] = wp
        self._last_name = name
        _save(self._wp)

        resp.success = True
        resp.message = f"Saved '{name}': x={wp['position']['x']:.3f}, y={wp['position']['y']:.3f}"
        self.get_logger().info(resp.message)
        return resp

    # ── delete last ─────────────────────────────────────

    def _handle_delete(self, req, resp):
        if not self._last_name or self._last_name not in self._wp:
            resp.success = False
            resp.message = "No waypoint to delete."
            return resp
        name = self._last_name
        del self._wp[name]
        _save(self._wp)
        # pick new last
        self._last_name = max(self._wp.keys()) if self._wp else None
        resp.success = True
        resp.message = f"Deleted '{name}'"
        self.get_logger().info(resp.message)
        return resp

    # ── list ────────────────────────────────────────────

    def _handle_list(self, req, resp):
        if not self._wp:
            resp.success = True
            resp.message = "No waypoints saved."
            return resp
        lines = [f"  {n}: x={w['position']['x']:.3f}, y={w['position']['y']:.3f}"
                 for n, w in self._wp.items()]
        resp.success = True
        resp.message = "\n".join(lines)
        self.get_logger().info(f"Waypoints:\n{resp.message}")
        return resp

    # ── navigate last ───────────────────────────────────

    def _handle_nav(self, req, resp):
        if not self._last_name or self._last_name not in self._wp:
            resp.success = False
            resp.message = "No waypoint to navigate to."
            return resp

        wp = self._wp[self._last_name]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp["position"]["x"]
        goal.pose.pose.position.y = wp["position"]["y"]
        goal.pose.pose.position.z = wp["position"]["z"]
        goal.pose.pose.orientation.x = wp["orientation"]["x"]
        goal.pose.pose.orientation.y = wp["orientation"]["y"]
        goal.pose.pose.orientation.z = wp["orientation"]["z"]
        goal.pose.pose.orientation.w = wp["orientation"]["w"]

        self._nav.wait_for_server()
        self._nav.send_goal_async(goal)
        resp.success = True
        resp.message = f"Nav goal sent for '{self._last_name}': x={wp['position']['x']:.3f}, y={wp['position']['y']:.3f}"
        self.get_logger().info(resp.message)
        return resp


def main():
    rclpy.init()
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
