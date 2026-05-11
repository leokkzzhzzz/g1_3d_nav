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

# 1. package.xml: ROS1 symlink
cd $WS/src/livox_ros_driver2-master
[ ! -f package.xml ] && ln -sf package_ROS1.xml package.xml

# 2. CATKIN_IGNORE: 跳过非必要包
cd $WS/src
for pkg in movebase pointcloud_to_laserscan ros_map_edit tool velocity_smoother_ema; do
    [ ! -f $pkg/CATKIN_IGNORE ] && touch $pkg/CATKIN_IGNORE
done

# 3. MID360 配置：host IP + 倒装 roll=180
CONFIG=$WS/src/livox_ros_driver2-master/config/MID360_config.json
sed -i "s/\"cmd_data_ip\".*/\"cmd_data_ip\" : \"192.168.123.164\",/" $CONFIG
sed -i "s/\"push_msg_ip\".*/\"push_msg_ip\" : \"192.168.123.164\",/" $CONFIG
sed -i "s/\"point_data_ip\".*/\"point_data_ip\" : \"192.168.123.164\",/" $CONFIG
sed -i "s/\"imu_data_ip\".*/\"imu_data_ip\" : \"192.168.123.164\",/" $CONFIG
sed -i "s/\"roll\": 0.0/\"roll\": 180.0/" $CONFIG

# 4. mapping.launch: 去掉 octomap_server 和 rviz（容器无 GUI）
LAUNCH=$WS/src/fastlio2/launch/mapping.launch
cat > $LAUNCH << '\''LAUNCHEOF'\''
<launch>
    <include file="\$(find livox_ros_driver2)/launch_ROS1/msg_MID360.launch"/>
    <rosparam command="load" file="\$(find fastlio)/config/mapping.yaml" />
    <node pkg="fastlio" type="map_builder_node" name="map_builder_node" output="screen"/>
</launch>
LAUNCHEOF

echo "Init done: $WS"
'

echo "Container: hongtu_mapper"
echo "进入: docker exec -it hongtu_mapper bash"
echo "编译: cd HongTu/G1Nav2D && catkin_make -DROS_EDITION=ROS1 -j1"
echo "建图: source HongTu/G1Nav2D/devel/setup.bash && roslaunch fastlio mapping.launch"
