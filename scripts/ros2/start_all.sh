#!/bin/bash
# G1 ROS2 全栈一键启动 — 容器内运行: bash /root/start_all.sh
# 对应 ROS1 start_nav.sh: roscore→LiDAR→FAST-LIO→open3d_loc→map_server→laserscan
set -e

# ── DDS 配置 ──────────────────────────────────────
mkdir -p /root/.ros
cat > /root/.ros/fastrtps_profiles.xml << 'XMLEOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles>
  <participant profile_name="default" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp_only</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
  <transport_descriptors>
    <transport_descriptor><transport_id>udp_only</transport_id><type>UDPv4</type></transport_descriptor>
  </transport_descriptors>
</profiles>
XMLEOF

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "[DDS] FastRTPS config written to /root/.ros/fastrtps_profiles.xml"

# ── 清理旧进程 ─────────────────────────────────────
pkill -f fastlio_mapping 2>/dev/null || true
pkill -f livox_ros_driver2_node 2>/dev/null || true
pkill -f global_localization_node 2>/dev/null || true
pkill -f map_server 2>/dev/null || true
pkill -f pointcloud_to_laserscan 2>/dev/null || true
sleep 2

# ── ROS2 环境 ──────────────────────────────────────
source /opt/ros/humble/setup.bash

# ── 重启 daemon ────────────────────────────────────
ros2 daemon stop 2>/dev/null || true
sleep 1
ros2 daemon start
sleep 2
echo "[DAEMON] ROS2 daemon ready"

# ── 1. LiDAR 驱动 ──────────────────────────────────
echo "[1/5] LiDAR driver..."
source /root/3d_nav_g1/livox_ws/install/setup.bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/driver.log 2>&1 &
sleep 5
grep -q "successfully enable" /tmp/driver.log && echo "  OK" || echo "  FAIL"

# ── 2. FAST-LIO ────────────────────────────────────
echo "[2/5] FAST-LIO..."
source /root/3d_nav_g1/g1_ws/install/setup.bash
nohup ros2 launch fast_lio mapping.launch.py rviz:=false > /tmp/fastlio.log 2>&1 &
sleep 8
grep -q "init finished" /tmp/fastlio.log && echo "  OK" || echo "  FAIL"
# Wait for odometry to start
for i in $(seq 1 10); do
  timeout 2 ros2 topic echo /Odometry_loc --once &>/dev/null && break
  sleep 2
done
echo "  Odometry flowing"

# ── 3. open3d_loc ──────────────────────────────────
echo "[3/5] open3d_loc (loading PCD ~40s)..."
nohup ros2 launch open3d_loc open3d_loc_g1.launch.py rviz:=false > /tmp/loc.log 2>&1 &
for i in $(seq 1 30); do
  sleep 5
  if timeout 2 ros2 topic echo /localization_3d --once &>/dev/null; then
    echo "  READY ($(($i*5))s)"
    break
  fi
  echo -n "."
done

# ── 4. map_server (2D 栅格地图) ─────────────────────
echo "[4/5] map_server..."
nohup ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=/root/maps/accumulated_grid.yaml \
    -r __node:=map_server \
    -r /map:=/map_2d > /tmp/mapserver.log 2>&1 &
sleep 3
ros2 lifecycle set /map_server configure 2>/dev/null
ros2 lifecycle set /map_server activate 2>/dev/null
echo "  OK"

# ── 5. pointcloud_to_laserscan (3D→2D, /scan) ─────
echo "[5/5] pointcloud_to_laserscan..."
nohup ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r __node:=pointcloud_to_laserscan \
    -p target_frame:=body \
    -p min_height:=0.15 \
    -p max_height:=0.25 \
    -p angle_min:=-1.5708 \
    -p angle_max:=1.5708 \
    -p angle_increment:=0.0087 \
    -p range_min:=0.2 \
    -p range_max:=20.0 \
    -r /cloud_in:=/cloud_registered_body_1 > /tmp/laserscan.log 2>&1 &
sleep 3
echo "  OK"

# ── 完成 ──────────────────────────────────────────
echo ""
echo "=== All 5 nodes started ==="
echo "  LiDAR → FAST-LIO → open3d_loc → map_server → laserscan"
echo ""
echo "Verify: ros2 node list"
echo "Rviz:   Fixed Frame=map, add /map /map_2d /scan /cloud_registered_body_1"
echo "Stop:   pkill -f ros2"
