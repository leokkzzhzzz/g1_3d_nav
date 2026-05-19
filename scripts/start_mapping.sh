#!/bin/bash
# ── Track 1a: ROS 1 离线建图 ──
docker run -it --rm     --network host     --name hongtu_mapper     -e DISPLAY=:0     -e MID360_IP=192.168.123.120     -v /tmp/.X11-unix:/tmp/.X11-unix:ro     -v $HOME/.Xauthority:/root/.Xauthority:ro     -e XAUTHORITY=/root/.Xauthority     -v $HOME/g1_3d_nav:/root/g1_3d_nav     hongtu-fastlio2:noetic     bash -c "
source /opt/ros/noetic/setup.bash
source /root/g1_3d_nav/deepglint_ws/devel/setup.bash
echo '=== Track 1a: Offline Mapping ==='
echo 'Ctrl-C to stop and auto-save scans.pcd'
roslaunch fast_lio mapping_g1_full.launch rviz:=false
"
