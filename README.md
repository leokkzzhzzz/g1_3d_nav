# G1 3D Navigation — v2.0.0

Unitree G1 人形机器人 3D OctoMap 导航系统部署。

## 当前进度

| Track | 内容 | 状态 |
|-------|------|------|
| **1a** | deepglint ROS 1 离线建图 | ✅ scans.pcd (129MB, 419万点) |
| **1b** | deepglint ROS 2 重定位 | ✅ 编译完成, ⬜ 测试 |
| 2 | jie_3d_nav OctoMap 导航 | ⬜ |
| 3 | g1pilot 控制器 | ⬜ |

## 架构

```
G1 Jetson Orin NX                         Leo 笔记本
┌──────────────────────────────┐          ┌─────────────────────┐
│ hongtu_mapper 容器            │  WiFi    │ g1_rviz 容器         │
│ (ROS 1 Noetic)               │ ◄────── │ (ROS 1 Noetic)       │
│                              │  11311   │ ROS_MASTER_URI → G1 │
│ deepglint livox_ros_driver2  │          │ rviz (官方配置)      │
│   → 点云 + IMU 同时 roll=180 │          └─────────────────────┘
│ fast_lio_mapping             │
│   → /livox/lidar CustomMsg   │          未来
│   → /livox/imu               │          ┌─────────────────────┐
│   → /cloud_registered_body   │ map.pcd  │ ROS 2 容器 (运行时)  │
│   → /Laser_map_1             │ ───────→ │ deepglint open3d_loc │
│   → TF (body)                │          │ octomap_server       │
│                              │          │ jie_3d_nav + Web     │
│ 一条命令:                     │          │ g1pilot 控制器       │
│ mapping_g1_full.launch       │          └─────────────────────┘
└──────────────────────────────┘
```

## 目录

| 文件 | 说明 |
|------|------|
| [CONTEXT.md](CONTEXT.md) | 领域术语、架构决策 (ADR) |
| [DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md) | 三路部署计划 |
| [docs/PRD_TRACK1A.md](docs/PRD_TRACK1A.md) | Track 1a 详细 PRD |
| [track1a/](track1a/) | Dockerfile + 启动脚本 |

## Track 1a: 离线建图 — 快速开始

### 前置条件

- G1 Jetson Orin NX（Ubuntu 22.04, Docker）
- Livox MID360（IP: 192.168.123.120, 倒装 roll=180°）
- Docker 镜像 `hongtu-fastlio2:noetic`

### G1 上构建镜像

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

### G1 上 clone 并编译 deepglint

```bash
mkdir -p ~/g1_3d_nav
cd ~/g1_3d_nav
git clone https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git deepglint_loc

# 启动容器
~/g1_3d_nav/start_hongtu.sh
docker exec -it hongtu_mapper bash

# 容器内
mkdir -p /root/g1_3d_nav/deepglint_ws/src
cd /root/g1_3d_nav/deepglint_ws/src
ln -sf ../../deepglint_loc/livox_ros_driver2 .
ln -sf ../../deepglint_loc/FAST_LIO .
touch open3d_loc/CATKIN_IGNORE  # 建图不需要定位模块
ln -sf ../../deepglint_loc/open3d_loc .
touch /root/g1_3d_nav/deepglint_ws/src/open3d_loc/CATKIN_IGNORE

# 修复包名和依赖
cd livox_ros_driver2
[ ! -f package.xml ] && ln -sf package_ROS1.xml package.xml

cd ../FAST_LIO
sed -i 's|livox_ros_driver|livox_ros_driver2|g' CMakeLists.txt package.xml
grep -rln "livox_ros_driver/" . | xargs -r sed -i 's|livox_ros_driver/|livox_ros_driver2/|g'
grep -rln "livox_ros_driver::" . | xargs -r sed -i 's|livox_ros_driver::|livox_ros_driver2::|g'

# 配置 MID360 IP
sed -i 's/192.168.123.222/192.168.123.164/g' \
    /root/g1_3d_nav/deepglint_ws/src/livox_ros_driver2/config/MID360_config.json

# 编译
source /opt/ros/noetic/setup.bash
cd /root/g1_3d_nav/deepglint_ws
catkin_make -DROS_EDITION=ROS1 -j1
make -C build fast_lio_generate_messages_cpp -j1  # 先生成消息头
make -C build -j1
```

### 建图

```bash
# G1 容器内 — 一键启动驱动 + FAST-LIO（无 RViz）
# PCD 已通过 mid360_g1.yaml 预配置为退出时自动保存
source /opt/ros/noetic/setup.bash
source /root/g1_3d_nav/deepglint_ws/devel/setup.bash
roslaunch fast_lio mapping_g1_full.launch rviz:=false
```

**Ctrl-C 停止时自动保存全部点云为一个 PCD**（`pcd_save_en: true`, `interval: -1`）。
保存在 `deepglint_ws/src/FAST_LIO/PCD/scans.pcd`。

配置项在 `deepglint_ws/src/FAST_LIO/config/mid360_g1.yaml`:
```yaml
pcd_save:
    pcd_save_en: true    # 改为 true
    interval: -1         # -1 = 所有帧合并为一个 PCD，退出时自动保存
```

### Leo 笔记本远程 RViz

```bash
# Leo 本机
xhost +local:
docker run -d --rm --network host --name g1_rviz \
    -e DISPLAY=:1 \
    -e ROS_MASTER_URI=http://<G1_WiFi_IP>:11311 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v $HOME/.Xauthority:/root/.Xauthority:ro \
    -e XAUTHORITY=/root/.Xauthority \
    ros:noetic-ros-base sleep infinity

docker exec g1_rviz bash -c 'apt-get update -qq && apt-get install -y -qq ros-noetic-rviz'

# 从 G1 拷贝官方 rviz 配置
ssh unitree@<G1_IP> "docker exec hongtu_mapper cat /root/g1_3d_nav/deepglint_ws/src/FAST_LIO/rviz_cfg/loam_livox.rviz" \
    | docker exec -i g1_rviz bash -c 'cat > /root/.rviz/loam_livox.rviz'

# 启动 RViz
docker exec g1_rviz bash -c 'source /opt/ros/noetic/setup.bash && rviz -d /root/.rviz/loam_livox.rviz'
```

### 建图流程

1. G1 站稳不动，启动建图，前 10 秒静止（IMU 初始化）
2. 遥控 G1 缓慢走一圈，走闭环路径触发回环检测
3. PCD 自动保存到 `PCD/` 目录
4. 复制 PCD 到 `/root/g1_3d_nav/maps/` 供后续 Track 使用

## 硬件

| 组件 | 型号 | IP |
|------|------|-----|
| 机器人 | Unitree G1 Edu | — |
| 板载计算机 | Jetson Orin NX | 192.168.123.164 (有线) / 192.168.100.30 (WiFi) |
| LiDAR | Livox MID360 (倒装) | 192.168.123.120 |
| 深度相机 | Intel D435i | — |

## Track 1b: ROS 2 重定位

### 镜像 `3d_nav_g1` (7.2GB)

```
/root/3d_nav_g1/
├── deps/open3d141/                    ← Open3D 1.4.1 (ARM64)
├── livox_ws/                          ← livox_ros_driver2 (驱动)
│   └── install/
├── g1_ws/                             ← 定位 workspace
│   └── install/
│       ├── fast_lio/fastlio_mapping   ← FAST-LIO2 里程计
│       └── open3d_loc/                ← ICP 全局重定位
├── maps/scans.pcd                     ← Track 1a 地图
├── start.sh                           ← 一键启动定位
└── start_driver.sh                    ← 启动 MID360 驱动
```

### 关键编译修复

| 问题 | 解决 |
|------|------|
| `CATKIN_IGNORE` 污染 | 从宿主机 `git archive` 干净导出 |
| `HUMBLE_ROS` 未设 | `--cmake-args -DHUMBLE_ROS=humble` |
| `liblapacke.so` 缺失 | `apt install liblapacke-dev` |
| numpy colcon 污染 | `touch /usr/lib/python3/numpy/COLCON_IGNORE` |
| livox 双 workspace | 驱动独立 `livox_ws`，先编译先 source |
| Open3D 预编译 | Baidu 网盘 `open3d141_arm.zip` |

- [deepglint FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID) — 建图驱动 + 运行时定位
- [HongTu (HongTu FAST-LIO2)](https://github.com/yuanqizhiti/HongTu) — 原始参考（已被 deepglint 替代）
- [jie_3d_nav](https://github.com/6-robot/jie_3d_nav) — 3D OctoMap 导航
- [g1pilot](https://github.com/hucebot/g1pilot) — G1 控制器

## 版本

| 版本 | 日期 | 内容 |
|------|------|------|
| v1.1.0 | 2026-05-12 | deepglint 驱动替代 HongTu, lidar2base_rpy 修正, 远程 RViz, PCD 参数保存 |
| v1.0.0 | 2026-05-11 | 初始架构文档 + HongTu 离线建图容器 |
