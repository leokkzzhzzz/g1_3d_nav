#!/bin/bash
# G1 宿主机上运行 — 将 ROS2 topic 桥接到 ROS1 roscore
# 前提: ROS1 容器 roscore 已启动, ROS2 容器节点已启动
#
# 用法: bash scripts/ros2/start_ros1_bridge.sh

set -e

ROS1_CONTAINER=3d_nav_ros1
ROS2_CONTAINER=3d_nav_ros2
BRIDGE_DIR=~/ros-humble-ros1-bridge

echo "=== G1 ROS1 Bridge ==="

# 1. 确保 ROS1 roscore 在运行
if ! docker exec $ROS1_CONTAINER bash -c 'source /opt/ros/noetic/setup.bash && rostopic list' &>/dev/null; then
    echo "[1/3] Starting ROS1 container + roscore..."
    docker start $ROS1_CONTAINER
    docker exec -d $ROS1_CONTAINER bash -c 'source /opt/ros/noetic/setup.bash && roscore'
    sleep 3
else
    echo "[1/3] ROS1 roscore already running"
fi

# 2. 确保 ROS2 容器和节点在运行
if ! docker exec $ROS2_CONTAINER bash -c 'source /opt/ros/humble/setup.bash && ros2 node list' &>/dev/null; then
    echo "[2/3] Starting ROS2 nodes..."
    docker start $ROS2_CONTAINER
    docker exec -d $ROS2_CONTAINER bash /root/start_all.sh
    echo "  Waiting 60s for nodes..."
    sleep 60
else
    echo "[2/3] ROS2 nodes already running"
fi

# 3. 启动 bridge
echo "[3/3] Starting dynamic_bridge..."
if [ ! -d "$BRIDGE_DIR/install" ]; then
    echo "ERROR: $BRIDGE_DIR not found. Build it first:"
    echo "  git clone https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder.git"
    echo "  cd ros-humble-ros1-bridge-builder"
    echo "  docker build . -t ros-humble-ros1-bridge-builder --network host"
    echo "  cd ~ && docker run --network host --rm ros-humble-ros1-bridge-builder | tar xvzf -"
    exit 1
fi

source /opt/ros/humble/setup.bash
source $BRIDGE_DIR/install/local_setup.bash
ROS_MASTER_URI=http://localhost:11311 \
    ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
