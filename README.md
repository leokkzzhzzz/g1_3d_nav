# G1 3D Navigation — v3.0.0

Unitree G1 人形机器人 3D 导航部署。**当前完成离线建图 + ROS 1 运行时重定位 + 远程 RViz。**

## 项目状态

| Track | 内容 | 状态 |
|-------|------|------|
| 1a | ROS 1 离线建图 → scans.ply | ✅ 完成 |
| 1b | ROS 1 重定位 + 远程 RViz (X11) | ✅ 完成 |
| 2 | jie_3d_nav OctoMap 导航 | ⬜ |
| 3 | g1pilot 控制器 | ⬜ |

## 待办

| # | 任务 | Track |
|---|------|-------|
| 1 | **远程 RViz 优化** — X11 转发 129MB 点云太卡，改为本地渲染（foxglove_bridge 已就绪，Foxglove Studio 浏览器端渲染比 X11 流畅） | 1b |
| 2 | **补齐 ROS 2 版功能** — `loc_map_cur.rviz` → RViz2 兼容格式；补充 `pointcloud_transformer_node` 可视化 topic；修复 `map` TF 帧缺失 | 1b |
| 3 | **定位精度量化评估** — 在多个已知位置记录 `/localization_3d` vs 真值，计算 ATE/RPE | 1b |
| 4 | **ROS 1/2 双轨长期策略** — Track 1a(建图) 保留 ROS 1；Track 1b(定位) 主用 ROS 1，逐步迁移至 ROS 2；Track 2/3 用 ROS 2 | 全部 |

## ROS 1 vs ROS 2 分析

| 维度 | ROS 1 (main 分支) | ROS 2 (humble 分支) |
|------|------------------|---------------------|
| 官方 rviz 配置 | `loc_map_cur.rviz` 直接可用 | 格式不兼容，类名全错 |
| `/map` topic | 发布 + TF 帧 `map` 完整 | 发布但 `map` 帧无 TF，RViz2 报错 |
| 可视化 topic | 完整（`/map`, `/submap`, `/scan2map`） | `pointcloud_transformer_node` 被注释 |
| 跨机器可视化 | TCPROS + rosbridge 可靠 | DDS 跨 WiFi TF 丢帧 |
| 结论 | ✅ 生产可用 | ⚠️ 待补齐功能 |

## 架构

```
G1 (Jetson Orin NX)
┌──────────────────────────────────────────────┐
│ hongtu_mapper 容器 (ROS 1 Noetic, network host)│
│                                              │
│ livox_ros_driver2 (MID360 驱动 @ 10Hz)        │
│   ↓ /livox/lidar                             │
│ fast_lio (FAST-LIO2 里程计)                    │
│   ↓ /Odometry_loc, /Laser_map_1              │
│ open3d_loc (ICP 全局重定位)                    │
│   ↓ /map (预建 129MB PCD), /localization_3d   │
│ foxglove_bridge (WebSocket ws://:9090)        │
│                                              │
│ ← X11 rviz 转发到 Leo                         │
└──────────────────────────────────────────────┘
```

## ROS 1 版本

### 架构

所有进程在 G1 的 `hongtu_mapper` 容器内运行（ROS 1 Noetic, network host）：

```
livox_ros_driver2 (MID360 驱动, 10Hz)
  → /livox/lidar (CustomMsg)
fast_lio (FAST-LIO2 里程计)
  → /Odometry_loc, /Laser_map_1, /cloud_registered_body_1
open3d_loc (ICP 全局重定位, 加载预建 PCD)
  → /map, /localization_3d, /localization_3d_confidence
foxglove_bridge (WebSocket ws://:9090)
  → Foxglove Studio 本地渲染（比 X11 流畅）
```

### 前提：容器已运行

```bash
docker run -d --network host --name hongtu_mapper \
    -e DISPLAY=:0 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v ~/g1_3d_nav/deepglint_ws:/root/deepglint_ws \
    -v ~/g1_3d_nav/deepglint_loc:/root/deepglint_loc \
    -v ~/g1_3d_nav/maps:/root/maps \
    hongtu-fastlio2:noetic sleep infinity
```

### 完整操作流程

**G1 终端 1 — MID360 驱动：**

```bash
docker exec -it hongtu_mapper bash -c 'source /opt/ros/noetic/setup.bash && source /root/deepglint_ws/devel/setup.bash && roslaunch livox_ros_driver2 msg_MID360.launch'
```

**G1 终端 2 — FAST-LIO（不开 RViz）：**

```bash
docker exec -it hongtu_mapper bash -c 'source /opt/ros/noetic/setup.bash && source /root/deepglint_ws/devel/setup.bash && roslaunch fast_lio mapping_mid360_g1.launch rviz:=false'
```

**G1 终端 3 — 3D 定位：**

```bash
docker exec -it hongtu_mapper bash -c 'source /opt/ros/noetic/setup.bash && source /root/deepglint_ws/devel/setup.bash && roslaunch open3d_loc open3d_loc_g1.launch'
```

> 等约 40 秒，看到日志 `initialize finished` 即加载 129MB PCD 完成。

### Leo 远程 RViz

#### 原理

G1 上所有 ROS 节点运行在一个容器内（`hongtu_mapper`），使用 `--network host` 暴露所有端口。Leo 本地开一个 ROS 1 容器，`ROS_MASTER_URI` 指向 G1 的 roscore。**数据走 WiFi（TCPROS），渲染走 Leo Intel GPU**——比 X11 转发帧率大幅提升。

```
G1 hongtu_mapper                              Leo leo_rviz
┌──────────────────────────┐                 ┌────────────────────────┐
│ roscore :11311           │   TCPROS (WiFi) │ ROS_MASTER_URI=G1:11311 │
│ /livox/lidar             │ ◄────────────── │ /etc/hosts: unitree-g1-nx │
│ /Odometry_loc            │   数据流          │ rviz -d loc_map_cur.rviz │
│ /map (129MB PCD)         │                 │ → Leo GPU 本地渲染      │
│ /Laser_map_1             │                 └────────────────────────┘
└──────────────────────────┘
```

#### 一次性准备（Leo）

```bash
# 创建 ROS 1 rviz 容器
docker run -d --name leo_rviz \
    -e DISPLAY=:1 \
    -e ROS_MASTER_URI=http://192.168.100.30:11311 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v /home/leo/g1_3d_nav_deploy/configs:/root/configs:ro \
    ros:noetic-ros-base sleep infinity

# 安装 rviz + 添加 G1 hostname 解析（关键）
docker exec leo_rviz apt-get update -qq
docker exec leo_rviz apt-get install -y -qq ros-noetic-rviz
docker exec leo_rviz bash -c 'echo "192.168.100.30 unitree-g1-nx" >> /etc/hosts'
```

> **为什么加 `/etc/hosts`**：ROS publisher 广播主机名 `unitree-g1-nx`，Leo 容器 DNS 无法解析 → 数据连不上。加 hosts 后 TCPROS 直连。

#### 启动 RViz

```bash
docker exec leo_rviz bash -c 'source /opt/ros/noetic/setup.bash && rviz -d /root/configs/loc_map_cur_leo.rviz'
```

### 重定位操作

1. RViz 中点击 **2D Pose Estimate**（绿色箭头）
2. 在地图点云上点击 G1 当前位置，拖动箭头对准前进方向
3. ICP 自动匹配，`/localization_3d_confidence` 输出置信度

### 验证定位

```bash
docker exec hongtu_mapper bash -c 'source /opt/ros/noetic/setup.bash && source /root/deepglint_ws/devel/setup.bash && timeout 5 rostopic echo /localization_3d_confidence -n 1'
# > 0.7 → 锁定；> 0.9 → 高精度
```

### 重定位操作

1. RViz 中点击 **2D Pose Estimate**（绿色箭头）
2. 在地图点云上点击 G1 当前位置，拖动箭头对准前进方向
3. ICP 自动匹配

### 验证定位精度

```bash
# 实时置信度（ICP fitness）
docker exec hongtu_mapper bash -c 'source /opt/ros/noetic/setup.bash && source /root/deepglint_ws/devel/setup.bash && rostopic echo /localization_3d_confidence'
# > 0.7 → 锁定；> 0.9 → 高精度

# 当前位姿
docker exec hongtu_mapper bash -c 'source /opt/ros/noetic/setup.bash && source /root/deepglint_ws/devel/setup.bash && timeout 5 rostopic echo /localization_3d -n 1'

# RViz 肉眼验证：/map（灰色预建地图）vs /cloud_registered_body_1（彩色实时点云）是否对齐
```

## 目录结构

```
~/g1_3d_nav/
├── deepglint_ws/              ← ROS 1 workspace (FAST-LIO + livox + open3d_loc)
│   ├── src/
│   │   ├── FAST_LIO -> ../../deepglint_loc/FAST_LIO
│   │   ├── livox_ros_driver2 -> ../../deepglint_loc/livox_ros_driver2
│   │   └── open3d_loc -> ../../deepglint_loc/open3d_loc
│   └── devel/lib/
│       ├── fast_lio/fastlio_mapping
│       ├── livox_ros_driver2/livox_ros_driver2_node
│       └── open3d_loc/global_localization_node
├── deepglint_loc/             ← deepglint 源码 (main 分支, ROS 1)
├── deps/open3d141/            ← Open3D v1.4.1 (ARM64 预编译)
├── maps/
│   ├── scans.ply (129MB)      ← Track 1a 建图输出
│   └── map.ply → scans.ply    ← 定位加载的地图
└── g1_ws/                     ← ROS 2 workspace (humble, 待补齐)
```

## Docker 镜像

| 镜像 | ROS | 大小 | 用途 |
|------|-----|------|------|
| `hongtu-fastlio2:noetic` | Noetic | 5.7GB | Track 1a+1b ROS 1 全部 |
| `3d_nav_g1` | Humble | 7.2GB | Track 2/3 (待用) |

## 硬件

| 组件 | 型号 | IP |
|------|------|-----|
| 机器人 | Unitree G1 Edu | — |
| 板载计算机 | Jetson Orin NX | 192.168.123.164 (有线) / 192.168.100.30 (WiFi) |
| LiDAR | Livox MID360 (倒装 roll=180°) | 192.168.123.120 |

## 相关仓库

- [g1_3d_nav](https://github.com/leokkzzhzzz/g1_3d_nav) — 本项目
- [deepglint FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID)
- [jie_3d_nav](https://github.com/6-robot/jie_3d_nav)
- [g1pilot](https://github.com/hucebot/g1pilot)

## 版本

| 版本 | 日期 | 内容 |
|------|------|------|
| v3.0.0 | 2026-05-14 | **切回 ROS 1 主方案** — open3d_loc 编译成功, loc_map_cur.rviz 官方可视化, 远程 X11 + Foxglove 双通道, 定位 conf=0.90 |
| v2.2.0 | 2026-05-13 | ROS 2 DDS 远程 RViz 实验通过 (已废弃) |
| v2.1.0 | 2026-05-13 | 全流程操作说明, start_mapping.sh |
| v2.0.0 | 2026-05-13 | Track 1b ROS 2 容器编译完成 |
| v1.0.0 | 2026-05-11 | 初始架构 + Track 1a |
