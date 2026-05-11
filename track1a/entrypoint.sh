#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash

# Workspace is mounted from host at /root/g1_3d_nav
WS=/root/g1_3d_nav
HONGTU_SRC=$WS/HongTu/G1Nav2D/src

if [ ! -d "$HONGTU_SRC" ]; then
    echo "ERROR: HongTu not found at $HONGTU_SRC"
    echo "Mount your host ~/g1_3d_nav to /root/g1_3d_nav"
    exit 1
fi

# Symlink workspace src → HongTu/G1Nav2D/src
if [ ! -L "$WS/src" ]; then
    ln -sf HongTu/G1Nav2D/src "$WS/src"
fi

# Source setup if already built
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"

echo "=== HongTu Fastlio2 Container ==="
echo "Workspace: $WS"
echo "MID360 IP: ${MID360_IP:-not set (update livox config manually)}"
echo ""
echo "If not yet built, run:"
echo "  cd /root/g1_3d_nav && catkin_make -DROS_EDITION=ROS1"
echo ""

exec "$@"
