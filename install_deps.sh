#!/bin/bash
# G1 ROS 1 全栈依赖安装
set -e

apt-get update

# ROS 基础
apt-get install -y ros-noetic-ros-base ros-noetic-rviz ros-noetic-tf2-tools

# PCL + Eigen
apt-get install -y libpcl-dev libeigen3-dev

# 导航
apt-get install -y ros-noetic-move-base ros-noetic-teb-local-planner \
    ros-noetic-global-planner ros-noetic-costmap-2d ros-noetic-map-server \
    ros-noetic-dwa-local-planner ros-noetic-navfn ros-noetic-costmap-converter

# Open3D (需手动编译 ARM64 版，见 docs/open3d_arm64.md)
# LAPACKE (需手动编译，见 docs/lapacke.md)

# 工具
apt-get install -y ros-noetic-topic-tools ros-noetic-rviz-plugin-tutorials

echo "ROS dependencies installed."
echo "Next: compile Open3D and LAPACKE for ARM64 (see docs/), then catkin_make"
