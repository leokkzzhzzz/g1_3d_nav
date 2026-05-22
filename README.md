# G1 3D Navigation — v3.5.0

Unitree G1 人形机器人 3D 导航部署。**离线建图 + 3D 重定位 + 2D 地图 + 导航 + 底盘控制。**

## 项目状态

| Track | 内容 | 状态 |
|-------|------|------|
| 1a | ROS 1 离线建图 → scans.pcd + 2D 栅格地图 | ✅ 完成 |
| 1b | ROS 1 重定位 + 远程 RViz + move_base 导航 | ✅ 完成 |
| 2 | TEB 替換為 G1 雙足控制器 | ⬜ |
| 3 | 镜像 CI/CD | ⬜ |

## 待办

| # | 任务 |
|---|------|
| 1 | **TEB 替换** — TEB 为轮式机器人设计，G1 双足需要"先转向→再直走"简单控制器 |
| 2 | **速度调参** — G1 最大步行 0.3-0.6 m/s，当前 TEB 参数需独立优化 |
| 3 | **控制器固化** — `bridge_and_control.py` 已从 /tmp 移入 home，重启不丢 |

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
│ hongtu_persistent 容器 (ROS 1 Noetic)         │
│                                              │
│ livox_ros_driver2 (MID360 驱动 @ 10Hz)        │
│   ↓ /livox/lidar                             │
│ fast_lio (FAST-LIO2 里程计)                    │
│   ↓ /Odometry_loc, /cloud_registered_body_1  │
│ open3d_loc (ICP 全局重定位, 加载 247M PCD)     │
│   ↓ /localization_3d, /map (PointCloud2)      │
│ map_server (2D 栅格地图)                       │
│   ↓ /map_2d                                   │
│                                              │
│ ← TCPROS (WiFi) → Leo RViz                   │
└──────────────────────────────────────────────┘
```

---

## 快速部署（ros1 分支）

### 1. 构建持久化镜像

基于 `hongtu-fastlio2:noetic`，固化所有 apt/pip 依赖，产出 `hongtu-persistent:latest`：

```bash
cd ~/g1_ros1_ws
docker build --network host -t hongtu-persistent:latest -f Dockerfile .
```

### 2. 准备宿主机目录

```bash
~/g1_ros1_ws/
├── src/deepglint/              ← FAST_LIO + livox_ros_driver2 + open3d_loc
├── src/navigation/             ← pointcloud_to_laserscan 等
├── devel/lib/                  ← 编译产物
├── Dockerfile                  ← 镜像构建文件
├── entrypoint.sh               ← Open3D 软链 + 启动
└── scripts/start_loc.sh        ← 一键启动脚本

~/g1_deps/open3d141/            ← Open3D C++ SDK（从 3d_nav_g1 镜像提取）

~/g1_3d_nav/maps/
├── scans.pcd                   ← 3D 全局点云 (247M)
├── accumulated_grid.pgm        ← 2D 栅格地图
└── accumulated_grid.yaml
```

### 3. 创建持久化容器

```bash
docker run -d \
    --runtime=runc \
    --network host \
    --name hongtu_persistent \
    --restart unless-stopped \
    -v ~/g1_ros1_ws:/root/g1_ros1_ws \
    -v ~/g1_deps:/root/g1_deps \
    -v ~/g1_3d_nav/maps:/root/maps \
    hongtu-persistent:latest \
    sleep infinity
```

### 4. 一键启动定位全栈

```bash
docker exec hongtu_persistent bash /root/g1_ros1_ws/scripts/start_loc.sh
```

脚本按顺序启动 4 个节点，每步自动验证 topic 上线：

| 步骤 | 节点 | 验证 topic |
|------|------|-----------|
| 1 | livox_ros_driver2 MID360 驱动 | `/livox/lidar` |
| 2 | FAST-LIO 里程计 | `/cloud_registered_body_1` |
| 3 | open3d_loc ICP 全局定位 | `/localization_3d` |
| 4 | map_server 2D 栅格地图 | `/map_2d` |

### 5. Topic 参考

| Topic | 类型 | 发布者 | 用途 |
|-------|------|--------|------|
| `/livox/lidar` | CustomMsg | livox_ros_driver2 | MID360 原始点云 |
| `/livox/imu` | Imu | livox_ros_driver2 | MID360 IMU |
| `/cloud_registered_body_1` | PointCloud2 | FAST-LIO | 实时 3D 点云 (body frame) |
| `/Odometry_loc` | Odometry | FAST-LIO | 里程计 |
| `/localization_3d` | PoseWithCovariance | open3d_loc | 全局定位结果 |
| `/map` | PointCloud2 | open3d_loc | 预建 3D PCD 地图 |
| `/submap` | PointCloud2 | open3d_loc | 局部子地图 |
| `/map_2d` | OccupancyGrid | map_server | 2D 栅格地图 |
| `/tf` | TF2 | open3d_loc + FAST-LIO | map→odom→camera_init→body |

### 6. 停止

```bash
# 停止容器
docker stop hongtu_persistent

# 只停内部进程（保留容器）
docker exec hongtu_persistent bash -c \
  "pkill -9 -f 'rosmaster|roslaunch|fastlio_mapping|global_localization_node|livox_ros_driver2_node|map_server'"
```

### 7. Leo 远程 RViz

```bash
docker start leo_rviz
xhost +local:
docker exec -e DISPLAY=:1 -e XAUTHORITY=/root/.Xauthority -e QT_X11_NO_MITSHM=1 \
  -e ROS_MASTER_URI=http://192.168.100.30:11311 leo_rviz bash -c '
source /opt/ros/noetic/setup.bash
rviz -d /root/maps/g1_navigation.rviz
'
```

RViz 显示配置：
- Fixed Frame → `map`
- Map → `/map_2d`（2D 栅格地图）
- PointCloud2 → `/map`（3D 静态 PCD）
- PointCloud2 → `/cloud_registered_body_1`（实时激光点云）

### Leo RViz 工作原理

G1 上所有 ROS 节点运行在一个容器内（`hongtu_persistent`），使用 `--network host` 暴露所有端口。Leo 本地开一个 ROS 1 容器，`ROS_MASTER_URI` 指向 G1 的 roscore。**数据走 WiFi（TCPROS），渲染走 Leo Intel GPU**。

```
G1 hongtu_persistent                         Leo leo_rviz
┌──────────────────────────┐                 ┌────────────────────────┐
│ roscore :11311           │   TCPROS (WiFi) │ ROS_MASTER_URI=G1:11311 │
│ /livox/lidar             │ ◄────────────── │ /etc/hosts: unitree-g1-nx│
│ /Odometry_loc            │   数据流         │ rviz -d g1_navigation   │
│ /map (247M PCD)          │                 │ → Leo GPU 本地渲染      │
│ /cloud_registered_body_1 │                 └────────────────────────┘
└──────────────────────────┘
```

### 重定位操作

1. RViz 中点击 **2D Pose Estimate**（绿色箭头）
2. 在地图点云上点击 G1 当前位置，拖动箭头对准前进方向
3. ICP 自动匹配，`/localization_3d_confidence` 输出置信度

### 验证定位

```bash
docker exec hongtu_persistent bash -c 'source /opt/ros/noetic/setup.bash && \
  source /root/g1_ros1_ws/devel/setup.bash && \
  timeout 5 rostopic echo /localization_3d_confidence -n 1'
# > 0.7 → 锁定；> 0.9 → 高精度
```

## 容器镜像

| 镜像 | ROS | 大小 | 用途 |
|------|-----|------|------|
| `hongtu-fastlio2:noetic` | Noetic | 5.7GB | 基镜像 |
| `hongtu-persistent:latest` | Noetic | ~5.7GB | 固化依赖的持久化版本 |
| `3d_nav_g1` | Humble | 7.2GB | ROS 2 导航 (待用) |

```bash
# 拉取基镜像
docker pull us-central1-docker.pkg.dev/dreamcontroltrain/g1-nav/hongtu-fastlio2:noetic
```

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
- [BotBrain](https://github.com/R-LFX/botbrain_project) — G1 导航 + 控制 (ROS 2)

## 版本

| 版本 | 日期 | 内容 |
|------|------|------|
| v3.5.0 | 2026-05-22 | **持久化部署** — Dockerfile + entrypoint 固化依赖, start_loc.sh 一键启动 4 步定位全栈, topic 自动验证 |
| v3.4.0 | 2026-05-15 | 导航全栈：move_base + DWA + velocity_smoother + bridge + SDK2 控制链, 镜像推送到 GCR |
| v3.0.0 | 2026-05-14 | **切回 ROS 1 主方案** — open3d_loc 编译成功, loc_map_cur.rviz 官方可视化, 远程 X11 + Foxglove 双通道, 定位 conf=0.90 |
| v2.2.0 | 2026-05-13 | ROS 2 DDS 远程 RViz 实验通过 (已废弃) |
| v2.1.0 | 2026-05-13 | 全流程操作说明, start_mapping.sh |
| v2.0.0 | 2026-05-13 | Track 1b ROS 2 容器编译完成 |
| v1.0.0 | 2026-05-11 | 初始架构 + Track 1a |
