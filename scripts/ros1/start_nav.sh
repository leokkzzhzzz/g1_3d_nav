#!/bin/bash
# G1 导航全链路启动
# 用法: bash start_nav.sh [需先 docker start hongtu_mapper]
set -e

C="docker exec hongtu_mapper bash -c"
S="source /opt/ros/noetic/setup.bash && source /root/deepglint_ws/devel/setup.bash"

echo '=== 1. roscore ==='
$C "$S && roscore" &
sleep 5

echo '=== 2. LiDAR ==='
$C "$S && roslaunch livox_ros_driver2 msg_MID360.launch" &
sleep 5

echo '=== 3. FAST-LIO ==='
$C "$S && roslaunch fast_lio mapping_mid360_g1.launch rviz:=false" &
sleep 12

echo '=== 4. open3d_loc ==='
$C "$S && roslaunch open3d_loc open3d_loc_g1.launch" &
sleep 25

echo '=== 5. 2D map ==='
$C "$S && rosrun map_server map_server /root/maps/accumulated_grid.yaml map:=/map_2d" &
sleep 2

echo '=== 6. 3D→2D scan ==='
$C "$S && rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/cloud_registered_body_1 _min_height:=-1 _max_height:=0.15 _range_max:=100" &
sleep 2

echo '=== 7. move_base (TEB+converter) ==='
$C "$S && roslaunch xju_pnc move_base.launch odom_topic:=/Odometry_loc" &
sleep 10

echo '=== 8. velocity_smoother ==='
$C "$S && roslaunch velocity_smoother_ema velocity_smoother_ema.launch" &
sleep 3

echo '=== 9. bridge_sender ==='
$C "$S && python3 /root/bridge_sender.py" &
sleep 3

echo '=== DONE ==='
$C "$S && rosnode list" | grep -c ''
echo 'nodes running'

echo ''
echo 'Host: bash start_nav_host.sh'
