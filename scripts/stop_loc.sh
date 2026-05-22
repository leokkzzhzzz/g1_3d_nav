#!/bin/bash
echo "=== 停止 ROS1 定位全栈 ==="
pkill -9 -f "rosmaster|roslaunch|rosout|rviz|fastlio_mapping|global_localization_node|livox_ros_driver2_node|map_server" 2>/dev/null
sleep 2
echo "已停止"
