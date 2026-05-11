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

# 等容器起来
sleep 2

# 初始化 workspace（仅首次需要）
docker exec hongtu_mapper bash -c '
[ ! -L /root/g1_3d_nav/src ] && ln -sf HongTu/G1Nav2D/src /root/g1_3d_nav/src
cd /root/g1_3d_nav/src
[ ! -f livox_ros_driver2-master/package.xml ] && ln -sf package_ROS1.xml livox_ros_driver2-master/package.xml
for pkg in movebase pointcloud_to_laserscan ros_map_edit tool velocity_smoother_ema; do
    [ ! -f $pkg/CATKIN_IGNORE ] && touch $pkg/CATKIN_IGNORE
done
echo "Workspace ready"
'

echo "Container started: hongtu_mapper"
echo "进入: docker exec -it hongtu_mapper bash"
echo "建图: source devel/setup.bash && roslaunch fastlio mapping.launch"
