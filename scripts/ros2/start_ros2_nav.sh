#!/bin/bash
# G1 ROS2 3D Navigation — full stack startup
# Runs inside 3d_nav_ros2 container (ROS2 Humble, host network)
#
# Prerequisites:
#   docker start 3d_nav_ros2
#
# Usage: bash start_ros2_nav.sh

set -e

CONTAINER=3d_nav_ros2
DDS_CONFIG=/tmp/fastrtps_docker.xml
MAP_PCD=/root/maps/scans_0518.pcd
MAP_YAML=/root/maps/accumulated_grid.yaml

# DDS transport fix for Docker (disable shared memory, UDP only)
export FASTRTPS_DEFAULT_PROFILES_FILE=file://$DDS_CONFIG

echo "=== G1 ROS2 Nav Stack ==="

# 1. LiDAR driver (MID360 @ 10Hz)
echo "[1/5] Starting LiDAR driver..."
docker exec -d $CONTAINER bash -c "
  source /opt/ros/humble/setup.bash
  source /root/3d_nav_g1/livox_ws/install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=file://$DDS_CONFIG
  ros2 launch livox_ros_driver2 msg_MID360_launch.py
"
sleep 3

# 2. FAST-LIO (LiDAR-inertial odometry)
echo "[2/5] Starting FAST-LIO..."
docker exec -d $CONTAINER bash -c "
  source /opt/ros/humble/setup.bash
  source /root/3d_nav_g1/g1_ws/install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=file://$DDS_CONFIG
  ros2 launch fast_lio mapping.launch.py rviz:=false
"
sleep 5

# 3. open3d_loc (ICP global localization against pre-built PCD)
echo "[3/5] Starting open3d_loc..."
docker exec -d $CONTAINER bash -c "
  source /opt/ros/humble/setup.bash
  source /root/3d_nav_g1/g1_ws/install/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=file://$DDS_CONFIG
  ros2 launch open3d_loc open3d_loc_g1.launch.py rviz:=false
"
sleep 8

# 4. Map server (2D grid map for Nav2)
echo "[4/5] Starting map_server..."
docker exec -d $CONTAINER bash -c "
  source /opt/ros/humble/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=file://$DDS_CONFIG
  ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=$MAP_YAML \
    -r __node:=map_server
"
sleep 2

# 5. Nav2 navigation stack (planner + controller + costmaps)
echo "[5/5] Starting Nav2..."
docker exec -d $CONTAINER bash -c "
  source /opt/ros/humble/setup.bash
  export FASTRTPS_DEFAULT_PROFILES_FILE=file://$DDS_CONFIG
  ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/root/nav2_params.yaml
"

echo ""
echo "=== All nodes started ==="
echo "Check: docker exec $CONTAINER bash -c 'source /opt/ros/humble/setup.bash && ros2 node list'"
echo "Send goal: ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: \"map\"}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}'"
