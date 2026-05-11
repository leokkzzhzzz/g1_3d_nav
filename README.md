# G1 3D Navigation — v1.0.0

Unitree G1 人形机器人 3D OctoMap 导航系统部署。

## 架构

```
ROS 1 容器 (离线建图)          ROS 2 容器 (运行时)
┌────────────────────┐        ┌──────────────────────────┐
│ HongTu fastlio2    │ map.pcd│ deepglint livox driver   │
│ + GTSAM 回环       │───────→│ FAST-LIO2 odometry       │
│ 产出: PCD 地图     │        │ open3d_loc 重定位        │
└────────────────────┘        │ octomap_server           │
                              │ jie_3d_nav 规划 + Web    │
                              │ g1pilot 控制器           │
                              └──────────────────────────┘
```

## 目录

| 文件 | 说明 |
|------|------|
| [CONTEXT.md](CONTEXT.md) | 领域术语、架构决策 (ADR) |
| [DEPLOYMENT_PLAN.md](DEPLOYMENT_PLAN.md) | 三路部署计划 |
| [track1a/](track1a/) | ROS 1 HongTu 离线建图容器 |

## Track 1a: 离线建图 — 快速开始 (G1 Jetson Orin NX)

### 构建镜像

```bash
cd track1a
./build.sh
```

如果 Docker build 失败（iptables raw 表缺失，Jetson 常见），用手动方式：

```bash
# 1. 拉取 base 并手动装依赖
docker pull ros:noetic-ros-base
docker run -d --network host --name hongtu_build ros:noetic-ros-base sleep infinity
docker exec hongtu_build apt-get update
docker exec hongtu_build apt-get install -y git build-essential cmake \
    libpcl-dev libeigen3-dev ros-noetic-gtsam ros-noetic-pcl-ros \
    ros-noetic-tf2 ros-noetic-tf2-ros ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs ros-noetic-geometry-msgs \
    ros-noetic-eigen-conversions ros-noetic-rviz libomp-dev libtbb-dev

# 2. 安装 Livox-SDK2
docker exec hongtu_build bash -c 'cd /tmp && \
    git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
    cd Livox-SDK2 && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local && make -j1 && make install'

# 3. Commit 镜像
docker stop hongtu_build && docker commit hongtu_build hongtu-fastlio2:noetic
```

### 启动容器

```bash
# 首次运行：clone HongTu 到工作区
mkdir -p ~/g1_3d_nav
git clone https://github.com/yuanqizhiti/HongTu.git ~/g1_3d_nav/HongTu

# 启动（自动挂载工作区，配置 MID360 IP: 192.168.123.120）
~/g1_3d_nav/start_hongtu.sh

# 进入容器
docker exec -it hongtu_mapper bash
```

### 容器内首次编译

```bash
source /opt/ros/noetic/setup.bash
cd /root/g1_3d_nav/HongTu/G1Nav2D

# 编译（单核 -j1，避免 Jetson OOM）
catkin_make -DROS_EDITION=ROS1 -j1
```

### 建图

```bash
source /root/g1_3d_nav/HongTu/G1Nav2D/devel/setup.bash
roslaunch fastlio mapping.launch

# 遥控 G1 扫描环境后保存
rosservice call /save_map "save_path: '/maps/target.pcd'"
```

PCD 产物保存到宿主机 `~/g1_3d_nav/maps/` 目录。

## 硬件

| 组件 | 型号 | IP |
|------|------|-----|
| 机器人 | Unitree G1 Edu | — |
| 板载计算机 | Jetson Orin NX | 192.168.123.164 |
| LiDAR | Livox MID360 (倒装) | 192.168.123.120 |
| 深度相机 | Intel D435i | — |

## 相关仓库

- [HongTu (HongTu FAST-LIO2)](https://github.com/yuanqizhiti/HongTu) — 离线建图
- [deepglint FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID) — 运行时定位
- [jie_3d_nav](https://github.com/6-robot/jie_3d_nav) — 3D OctoMap 导航
- [g1pilot](https://github.com/hucebot/g1pilot) — G1 控制器

## 版本

v1.0.0 — Track 1a 离线建图容器 + 架构文档
