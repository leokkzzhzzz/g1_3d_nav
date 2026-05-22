#!/bin/bash
set -e

WS=/root/g1_ros1_ws
source /opt/ros/noetic/setup.bash
source $WS/devel/setup.bash

check_topic() {
    local topic=$1 desc=$2
    for i in $(seq 1 20); do
        rostopic info $topic > /dev/null 2>&1 && echo "  [OK] $desc" && return 0
        sleep 1
    done
    echo "  [FAIL] $desc"
    return 1
}

echo "=== G1 ROS1 定位全栈启动 ==="

echo "[0/4] 清理旧进程 & 检查 PCD..."
pkill -9 -f "rosmaster|roslaunch|rosout|rviz|fastlio_mapping|global_localization_node|livox_ros_driver2_node|map_server" 2>/dev/null || true
sleep 2

PCD_PATH=/root/deepglint_loc/FAST_LIO/PCD/scans.pcd
if [ ! -f "$PCD_PATH" ]; then
    mkdir -p $(dirname "$PCD_PATH")
    cp /root/maps/scans.pcd "$PCD_PATH"
fi
echo "  PCD: $(ls -lh $PCD_PATH | awk "{print \$5}")"

echo "[1/4] LiDAR 驱动..."
roslaunch livox_ros_driver2 msg_MID360.launch &
sleep 6
check_topic /livox/lidar "LiDAR"

echo "[2/4] FAST-LIO..."
roslaunch fast_lio mapping_mid360_g1.launch rviz:=false &
sleep 10
check_topic /cloud_registered_body_1 "FAST-LIO"

echo "[3/4] open3d_loc ICP 定位 (247M PCD ~20s)..."
roslaunch open3d_loc open3d_loc_g1.launch &
sleep 22
check_topic /localization_3d "open3d_loc"

echo "[4/4] map_server (2D /map_2d)..."
rosrun map_server map_server /root/maps/accumulated_grid.yaml map:=/map_2d &
sleep 3
check_topic /map_2d "map_server"

echo ""
echo "=== 启动完成，topic 汇总 ==="
rostopic list 2>/dev/null | grep -E "livox|lidar|Odometry|cloud_registered|localization|map|tf$" | sort
echo ""
echo "RViz: Fixed Frame=map, Map→/map_2d, PointCloud2→/map, PointCloud2→/cloud_registered_body_1"
