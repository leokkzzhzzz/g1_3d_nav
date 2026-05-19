#!/bin/bash
# G1 ROS2 全栈一键启动 — 在容器内运行: bash start_all.sh
set -e

# ── DDS 配置 ──────────────────────────────────────
# FastRTPS 禁 shared memory，只走 UDP（Docker 必须）
cat > /tmp/fastrtps_docker.xml << 'XMLEOF'
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
# 同时写到 FastRTPS 默认路径，子进程自动加载，重启不丢
mkdir -p /root/.ros
cp /tmp/fastrtps_docker.xml /root/.ros/fastrtps_profiles.xml
export FASTRTPS_DEFAULT_PROFILES_FILE=file:///tmp/fastrtps_docker.xml
echo "[DDS] Config written to /root/.ros/fastrtps_profiles.xml"

# ── 清理旧进程 ─────────────────────────────────────
pkill -f fastlio_mapping 2>/dev/null || true
pkill -f livox_ros_driver2_node 2>/dev/null || true
pkill -f global_localization_node 2>/dev/null || true
sleep 1

# ── ROS2 环境 ──────────────────────────────────────
source /opt/ros/humble/setup.bash

# ── 1. LiDAR 驱动 ──────────────────────────────────
echo "[1/3] Starting LiDAR driver..."
source /root/3d_nav_g1/livox_ws/install/setup.bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py > /tmp/driver.log 2>&1 &
sleep 4
grep -q "successfully enable" /tmp/driver.log && echo "       LiDAR driver OK" || echo "       LiDAR driver FAIL"

# ── 2. FAST-LIO ────────────────────────────────────
echo "[2/3] Starting FAST-LIO..."
source /root/3d_nav_g1/g1_ws/install/setup.bash
nohup ros2 launch fast_lio mapping.launch.py rviz:=false > /tmp/fastlio.log 2>&1 &
sleep 6
grep -q "init finished" /tmp/fastlio.log && echo "       FAST-LIO OK" || echo "       FAST-LIO FAIL"

# ── 3. open3d_loc ──────────────────────────────────
echo "[3/3] Starting open3d_loc (loading PCD, ~40s)..."
nohup ros2 launch open3d_loc open3d_loc_g1.launch.py rviz:=false > /tmp/loc.log 2>&1 &
sleep 5
echo "       Waiting for localization init..."

# ── 等待定位上线 ────────────────────────────────────
for i in $(seq 1 20); do
    sleep 5
    if timeout 2 ros2 topic echo /localization_3d --once &>/dev/null; then
        echo "       open3d_loc READY"
        break
    fi
    echo "       ... waiting ($((i*5))s)"
done

echo ""
echo "=== All nodes started ==="
echo "Check: ros2 node list"
echo "TF:    ros2 topic echo /tf --once | grep frame_id"
echo "Stop:  pkill -f ros2"
