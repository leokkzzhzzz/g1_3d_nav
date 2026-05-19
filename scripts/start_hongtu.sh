#!/bin/bash
# ── HongTu Fastlio2 建图容器启动脚本 ──

docker run -d --rm \
    --network host \
    --name hongtu_mapper \
    -e DISPLAY=:0 \
    -e MID360_IP=192.168.123.120 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v $HOME/g1_3d_nav:/root/g1_3d_nav \
    hongtu-fastlio2:noetic \
    sleep infinity

sleep 2

# 初始化（仅首次需要）
docker exec hongtu_mapper bash -c '
WS=/root/g1_3d_nav/HongTu/G1Nav2D
cd $WS/src
[ ! -f livox_ros_driver2-master/package.xml ] && ln -sf package_ROS1.xml livox_ros_driver2-master/package.xml
for pkg in movebase pointcloud_to_laserscan ros_map_edit tool velocity_smoother_ema; do
    [ ! -f $pkg/CATKIN_IGNORE ] && touch $pkg/CATKIN_IGNORE
done
echo "Workspace ready: $WS"
'

echo "Container: hongtu_mapper"
echo "进入: docker exec -it hongtu_mapper bash"
echo "编译: cd HongTu/G1Nav2D && catkin_make -DROS_EDITION=ROS1 -j1"
echo "建图: source HongTu/G1Nav2D/devel/setup.bash && roslaunch fastlio mapping.launch"
