# ROS1 Bridge: 跨机器 ROS2 数据验证方案

## 问题

ROS2 DDS 跨 Tailscale/WiFi 传输不稳定——topic 名能发现但数据不通。根因是 Cyclone DDS 在多网卡（WiFi + Tailscale + Docker）环境下 peer 连接选错接口，UDP 数据通道建立失败。

ROS1 的 roscore TCP 中心化架构不存在此问题。

## 方案

在 G1 宿主机上运行 `ros1_bridge`，将 ROS2 topic 映射到 ROS1，Leo 通过已验证的 ROS1 roscore TCP 链路查看数据。

```
G1 宿主机
  ┌─────────────────────────────────────────┐
  │                                         │
  │  ~/ros-humble-ros1-bridge/              │
  │  dynamic_bridge                         │
  │    ├── ROS2 侧: DDS 发现 (本地)          │
  │    └── ROS1 侧: TCP → roscore:11311     │
  │                                         │
  └────────────┬──────────────┬─────────────┘
               │ DDS          │ TCP
               ▼              ▼
  ┌──────────────────┐  ┌──────────────────┐
  │ ROS2 容器         │  │ ROS1 容器         │
  │ (3d_nav_ros2)    │  │ (3d_nav_ros1)    │
  │                  │  │                  │
  │ LiDAR 驱动       │  │ roscore :11311   │
  │ FAST-LIO         │  │                  │
  │ open3d_loc       │  └────────┬─────────┘
  │ map_server       │           │ TCP
  │ laserscan        │           ▼
  └──────────────────┘  ┌──────────────────┐
                        │ Leo              │
                        │                  │
                        │ ROS1 RViz 容器    │
                        │ ROS_MASTER_URI=  │
                        │   100.76.79.32   │
                        └──────────────────┘
```

## 构建（一次性）

基于 [ros-humble-ros1-bridge-builder](https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder)，该仓库在 Docker 内同时安装 ROS1 Noetic + ROS2 Humble，编译 `ros1_bridge`，输出预编译包。

G1 宿主机已有 `ros-humble-desktop`，满足运行条件。

```bash
# SSH 到 G1
ssh unitree@100.76.79.32

# 克隆构建仓库
git clone https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder.git
cd ros-humble-ros1-bridge-builder

# 构建 Docker 镜像（Jetson arm64, 约 10 分钟）
docker build . -t ros-humble-ros1-bridge-builder --network host

# 提取编译好的 bridge 包
cd ~
docker run --network host --rm ros-humble-ros1-bridge-builder | tar xvzf -
# → ~/ros-humble-ros1-bridge/
```

## 启动（每次）

```bash
# 1. ROS1 容器 + roscore
docker start 3d_nav_ros1
docker exec -d 3d_nav_ros1 bash -c 'source /opt/ros/noetic/setup.bash && roscore'

# 2. ROS2 容器 + 全栈节点
docker start 3d_nav_ros2
docker exec 3d_nav_ros2 bash /root/start_all.sh  # 等待约 60s

# 3. Bridge
source ~/ros-humble-ros1-bridge/install/local_setup.bash
ROS_MASTER_URI=http://localhost:11311 \
  ros2 run ros1_bridge dynamic_bridge --bridge-all-2to1-topics
```

## Leo 查看

```bash
docker start leo_rviz
docker exec leo_rviz bash -c '
  export ROS_MASTER_URI=http://100.76.79.32:11311
  source /opt/ros/noetic/setup.bash
  rviz -d /root/maps/g1_navigation.rviz
'
```

## 局限

- 仅用于验证 ROS2 数据正确性，非生产方案
- `--bridge-all-2to1-topics` 桥接所有 topic，可能有不需要的
- 长期方案：DDS 多网卡问题解决后，直接用 ROS2 原生通信
