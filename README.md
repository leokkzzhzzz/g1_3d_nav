# G1 3D Navigation — v2.1.0

Unitree G1 人形机器人 3D 导航部署。当前完成离线建图 + 运行时重定位。

## 项目状态

| Track | 内容 | 状态 |
|-------|------|------|
| 1a | ROS 1 离线建图 → scans.pcd | ✅ 完成 |
| 1b | ROS 2 重定位 | ✅ 编译完成 |
| 2 | jie_3d_nav OctoMap 导航 | ⬜ |
| 3 | g1pilot 控制器 | ⬜ |

## 最终目录结构

G1 宿主机 `~/g1_3d_nav/`：

```
~/g1_3d_nav/
├── deepglint_ws/              ← ROS 1 建图 workspace
│   ├── devel/lib/fast_lio/fastlio_mapping
│   └── src/FAST_LIO/launch/
│       ├── mapping_mid360_g1.launch
│       └── mapping_g1_full.launch
├── livox_ws/                  ← ROS 2 驱动 workspace
│   └── install/livox_ros_driver2/
├── g1_ws/                     ← ROS 2 定位 workspace
│   └── install/
│       ├── fast_lio/fastlio_mapping
│       └── open3d_loc/global_localization_node
├── deps/open3d141/            ← Open3D v1.4.1 (ARM64)
├── maps/scans.pcd             ← 共享 PCD 地图 (129MB)
├── start_mapping.sh           ← Track 1a 一键建图
└── track1a/                   ← Dockerfile + 部署文档
```

## Track 1a: 离线建图

### 前置条件

- G1 Jetson Orin NX
- Docker 镜像 `hongtu-fastlio2:noetic` (5.7GB)
- Livox MID360 (IP: 192.168.123.120, 倒装 roll=180°)

### 一键启动

```bash
cd ~/g1_3d_nav
./start_mapping.sh
```

### 操作步骤

1. 启动后前 10 秒保持 G1 静止（IMU 初始化）
2. 遥控 G1 缓慢走一圈，走闭环路径触发回环
3. **Ctrl-C 停止 → 自动保存 `scans.pcd`** 到 `deepglint_ws/src/FAST_LIO/PCD/`

## Track 1b: 运行时重定位

### 前置条件

- Docker 镜像 `3d_nav_g1` (7.2GB)
- Track 1a 产出的 `scans.pcd`

### 启动容器

```bash
docker run -d --network host --name 3d_nav \
    -v $HOME/g1_3d_nav:/root/3d_nav_g1 \
    -e DISPLAY=:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    -e XAUTHORITY=/root/.Xauthority \
    3d_nav_g1 sleep infinity
```

### 启动定位

需要两个终端，都进容器：

**终端 1 — MID360 驱动：**

```bash
docker exec -it 3d_nav bash
source /opt/ros/humble/setup.bash
source /root/3d_nav_g1/livox_ws/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**终端 2 — 定位 + RViz：**

```bash
docker exec -it 3d_nav bash
source /opt/ros/humble/setup.bash
source /root/3d_nav_g1/livox_ws/install/setup.bash
source /root/3d_nav_g1/g1_ws/install/setup.bash
ros2 launch open3d_loc localization_3d_g1.launch.py
```

### 重定位操作

1. RViz 中点击 **2D Pose Estimate**（绿色箭头）
2. 在地图点云上点击 G1 当前位置，拖动箭头对准前进方向
3. ICP 自动匹配，`/localization_3d_confidence` 输出置信度

### 验证定位

```bash
docker exec 3d_nav bash -c 'source /opt/ros/humble/setup.bash && ros2 topic echo /localization_3d_confidence --once'
# data: 0.78 → 定位成功
```

## 镜像构建记录

### hongtu-fastlio2:noetic (5.7GB)

Docker build 在 Jetson 上失败（iptables raw 表缺失），需手动构建：

```bash
docker pull ros:noetic-ros-base
docker run -d --network host --name hongtu_build ros:noetic-ros-base sleep infinity
docker exec hongtu_build apt-get update
docker exec hongtu_build apt-get install -y git build-essential cmake \
    libpcl-dev libeigen3-dev ros-noetic-gtsam ros-noetic-pcl-ros \
    ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs ros-noetic-geometry-msgs \
    ros-noetic-eigen-conversions ros-noetic-rviz libomp-dev libtbb-dev

# Livox-SDK2
docker exec hongtu_build bash -c 'cd /tmp && \
    git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j1 && make install'

docker stop hongtu_build && docker commit hongtu_build hongtu-fastlio2:noetic
```

### 3d_nav_g1 (7.2GB)

从 `ros:humble-ros-base` (ARM64) 手动构建：

```bash
docker pull ros:humble-ros-base
docker run -d --network host --name g1_3dnav_build ros:humble-ros-base sleep infinity
docker exec g1_3dnav_build apt-get update
docker exec g1_3dnav_build apt-get install -y libeigen3-dev libpcl-dev \
    libc++-dev libc++abi-dev python3-colcon-common-extensions \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-nav-msgs \
    ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-pcl-ros \
    ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs \
    ros-humble-cv-bridge ros-humble-image-transport ros-humble-urdf \
    ros-humble-rviz2 ros-humble-rmw-cyclonedds-cpp \
    liblapacke-dev libyaml-cpp-dev git unzip

# Livox-SDK2 + Open3D + deepglint Humble 分支源码（略，见 track1a/ 文档）

docker stop g1_3dnav_build && docker commit g1_3dnav_build 3d_nav_g1
```

### 关键编译修复

| 问题 | 解决 |
|------|------|
| OOM (ARM64 8GB) | `-j1` 单核编译 |
| `HUMBLE_ROS` 未设 | `--cmake-args -DHUMBLE_ROS=humble` |
| `liblapacke.so` 缺失 | `apt install liblapacke-dev` |
| numpy 污染 colcon | `touch /usr/lib/python3/numpy/COLCON_IGNORE` |
| `CATKIN_IGNORE` 从 ROS1 污染 | 用 `git archive` 干净导出 |
| LiDAR topic 不匹配 | FAST-LIO config 用 `/livox/lidar` |
| PCD 路径 | `open3d_loc_g1.launch.py` 指向 `/root/3d_nav_g1/maps/scans.pcd` |

## 硬件

| 组件 | 型号 | IP |
|------|------|-----|
| 机器人 | Unitree G1 Edu | — |
| 板载计算机 | Jetson Orin NX | 192.168.123.164 (有线) / 192.168.100.30 (WiFi) |
| LiDAR | Livox MID360 (倒装) | 192.168.123.120 |
| 深度相机 | Intel D435i | — |

## 相关仓库

- [deepglint FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)
- [jie_3d_nav](https://github.com/6-robot/jie_3d_nav)
- [g1pilot](https://github.com/hucebot/g1pilot)

## 版本

| 版本 | 日期 | 内容 |
|------|------|------|
| v2.1.0 | 2026-05-13 | 全流程操作说明，start_mapping.sh |
| v2.0.0 | 2026-05-13 | Track 1b ROS 2 容器编译完成 |
| v1.1.0 | 2026-05-12 | deepglint 驱动替代 HongTu，远程 RViz |
| v1.0.0 | 2026-05-11 | 初始架构 + Track 1a |
