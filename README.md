# G1 3D Navigation — v2.2.0

Unitree G1 人形机器人 3D 导航部署。**当前完成离线建图 + 运行时重定位 + 远程 RViz。**

## 项目状态

| Track | 内容 | 状态 |
|-------|------|------|
| 1a | ROS 1 离线建图 → scans.pcd | ✅ 完成 |
| 1b | ROS 2 重定位 + Leo 远程 RViz | ✅ 完成 |
| 2 | jie_3d_nav OctoMap 导航 | ⬜ |
| 3 | g1pilot 控制器 | ⬜ |

## 架构

```
G1 (Jetson Orin NX)                     Leo 笔记本
┌────────────────────────────┐         ┌──────────────────────┐
│ 3d_nav 容器 (ROS 2 Humble) │ DDS UDP │ g1_rviz2 容器         │
│ ROS_DOMAIN_ID=77           │ ◄────── │ ROS_DOMAIN_ID=77      │
│                            │         │ rviz2 (官方配置)       │
│ livox_ros_driver2 (驱动)    │         └──────────────────────┘
│ fast_lio (FAST-LIO2 里程计)│
│ open3d_loc (ICP 全局重定位) │
│        ↓                   │
│ /localization_3d (位姿)    │
│ /Laser_map_1 (全局地图)    │
│ /cloud_registered_body_1   │
└────────────────────────────┘
```

## 快速开始

### Track 1a: 离线建图

```bash
~/g1_3d_nav/start_mapping.sh
# Ctrl-C 自动保存 scans.pcd
```

### Track 1b: 重定位 + 远程 RViz

**1. G1 终端 1 — MID360 驱动：**

```bash
docker exec -it 3d_nav bash -c 'export ROS_DOMAIN_ID=77 && source /opt/ros/humble/setup.bash && source /root/3d_nav_g1/livox_ws/install/setup.bash && ros2 launch livox_ros_driver2 msg_MID360_launch.py'
```

**2. G1 终端 2 — 定位（不开 RViz）：**

```bash
docker exec -it 3d_nav bash -c 'export ROS_DOMAIN_ID=77 && source /opt/ros/humble/setup.bash && source /root/3d_nav_g1/livox_ws/install/setup.bash && source /root/3d_nav_g1/g1_ws/install/setup.bash && ros2 launch open3d_loc localization_3d_g1.launch.py rviz:=false'
```

**3. 验证 DDS 连通（Leo）：**

```bash
docker exec g1_rviz2 bash -c 'source /opt/ros/humble/setup.bash && ros2 topic list | grep -iE "laser|locali|odom_loc"'
# 预期看到: /Laser_map_1, /localization_3d, /Odometry_loc
```

**4. Leo 开 RViz：**

```bash
docker exec g1_rviz2 bash -c 'source /opt/ros/humble/setup.bash && rviz2 -d /root/.rviz2/loc_map_cur.rviz'
```

**5. 重定位：** RViz 里点击 **2D Pose Estimate**（绿色箭头）→ 在地图上点 G1 位置 → 拖箭头对准方向。

**6. 验证定位：**

```bash
docker exec 3d_nav bash -c 'export ROS_DOMAIN_ID=77 && source /opt/ros/humble/setup.bash && ros2 topic echo /localization_3d_confidence --once'
# data: 1.0 → 定位锁定
```

## Leo 远程 RViz（一次性准备）

```bash
# rviz2 容器
docker run -d --network host --name g1_rviz2 \
    -e DISPLAY=:1 -e ROS_DOMAIN_ID=77 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    ros:humble-ros-base sleep infinity
docker exec g1_rviz2 bash -c 'apt-get update -qq && apt-get install -y -qq ros-humble-rviz2'

# 官方 rviz 配置（从 G1 拷过来）
# ssh unitree@192.168.100.30 "cat ~/g1_3d_nav/deepglint_loc/open3d_loc/rviz_cfg/loc_map_cur.rviz" \
#     | docker exec -i g1_rviz2 bash -c 'mkdir -p /root/.rviz2 && cat > /root/.rviz2/loc_map_cur.rviz'
```

## 目录结构

```
~/g1_3d_nav/
├── deepglint_ws/              ← ROS 1 建图 workspace
│   └── devel/lib/fast_lio/fastlio_mapping
├── livox_ws/                  ← ROS 2 驱动 workspace
├── g1_ws/                     ← ROS 2 定位 workspace
├── deps/open3d141/            ← Open3D v1.4.1 (ARM64)
├── maps/scans.pcd             ← 共享 PCD 地图
└── start_mapping.sh           ← Track 1a 一键建图
```

## Docker 镜像

| 镜像 | ROS | 大小 | 用途 |
|------|-----|------|------|
| `hongtu-fastlio2:noetic` | Noetic | 5.7GB | Track 1a 离线建图 |
| `3d_nav_g1` | Humble | 7.2GB | Track 1b 运行时定位 |

## 硬件

| 组件 | 型号 | IP |
|------|------|-----|
| 机器人 | Unitree G1 Edu | — |
| 板载计算机 | Jetson Orin NX | 192.168.123.164 (有线) / 192.168.100.30 (WiFi) |
| LiDAR | Livox MID360 (倒装 roll=180°) | 192.168.123.120 |

## 相关仓库

- [deepglint FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)
- [jie_3d_nav](https://github.com/6-robot/jie_3d_nav)
- [g1pilot](https://github.com/hucebot/g1pilot)

## 版本

| 版本 | 日期 | 内容 |
|------|------|------|
| v2.2.0 | 2026-05-13 | Leo 远程 RViz 通过 DDS domain 77, loc_map_cur.rviz |
| v2.1.0 | 2026-05-13 | 全流程操作说明, start_mapping.sh |
| v2.0.0 | 2026-05-13 | Track 1b ROS 2 容器编译完成 |
| v1.0.0 | 2026-05-11 | 初始架构 + Track 1a |
