# G1 3D Navigation — ROS 2

Unitree G1 人形机器人 ROS 2 Humble 3D 导航。**离线建图 + 3D ICP 重定位 + Nav2 导航 + SDK2 底盘控制。**

## 架构

```
G1 (Jetson Orin NX)
┌──────────────────────────────────────────────────┐
│ 3d_nav_ros2 容器 (ROS 2 Humble, network host)     │
│                                                  │
│ livox_ros_driver2 (MID360 驱动 @ 10Hz)            │
│   ↓ /livox/lidar                                 │
│ fast_lio (FAST-LIO2 里程计 @ 10Hz)                │
│   ↓ /Odometry_loc, /cloud_registered_body_1       │
│ open3d_loc (ICP 全局重定位 @ 3Hz)                  │
│   ↓ /tf (map→odom), /localization_3d              │
│ map_server (2D 栅格地图)                           │
│   ↓ /map                                          │
│ Nav2 (planner + RegulatedPurePursuit controller)  │
│   ↓ /cmd_vel                                      │
└──────────────────────────────────────────────────┘
                    ↓ TCP
┌──────────────────────────────────────────────────┐
│ G1 Host (Python 3.10 + Unitree SDK2)              │
│ cmd_vel_to_sdk2.py                               │
│   Twist → LocoClient.Move(vx, vy, vyaw)          │
└──────────────────────────────────────────────────┘
```

## TF 树

```
map ──→ odom ──→ camera_init ──→ body ──→ base_link ──→ motion_link
(open3d)  (static)  (FAST-LIO)           (static)       (static)
```

## 目录结构 (本分支)

```
g1_3d_nav_deploy/
├── README_ROS2.md                       ← 本文件
├── config/ros2/
│   ├── nav2_params.yaml                 ← Nav2 参数 (基于原版, G1 改动注释标出)
│   ├── loc_param_g1.yaml               ← open3d_loc 参数
│   ├── open3d_loc_g1.launch.py         ← open3d_loc 启动文件
│   └── fastrtps_docker.xml             ← DDS UDP-only 配置 (Docker 必须)
├── g1_control/
│   └── cmd_vel_to_sdk2.py              ← Twist → Unitree SDK2 控制器
├── ros1_ros2_bridge/
│   ├── bridge_sender.py                ← ROS 1 → TCP bridge (容器端)
│   └── bridge_receiver.py              ← TCP → ROS 2 bridge 接收端
├── scripts/ros2/
│   └── start_ros2_nav.sh               ← 一键启动脚本
└── grid_accumulator/                    ← 3D → 2D 栅格累积器 (建图用)
```

## Docker 镜像

| 镜像 | ROS | 大小 | 用途 |
|------|-----|------|------|
| `3d_nav_g1:latest` | Humble | 7.2GB | ROS 2 全栈 (LiDAR + FAST-LIO + open3d_loc + Nav2) |
| `hongtu-fastlio2:noetic` | Noetic | 5.7GB | ROS 1 (建图用) |

### 拉取镜像

```bash
# ROS 2 镜像 (本分支)
docker pull us-central1-docker.pkg.dev/dreamcontroltrain/g1-nav/3d_nav_g1:latest

# ROS 1 镜像 (建图用, main 分支)
docker pull us-central1-docker.pkg.dev/dreamcontroltrain/g1-nav/hongtu-fastlio2:noetic
```

### 创建容器

```bash
docker run -d --network host --name 3d_nav_ros2 \
    -v ~/g1_3d_nav/maps:/root/maps \
    us-central1-docker.pkg.dev/dreamcontroltrain/g1-nav/3d_nav_g1:latest sleep infinity
```

> `--network host` 必须, 否则 DDS 无法跨进程通信.

## 前置: DDS 配置 (Docker 必须)

FastRTPS 默认启用 shared memory 传输. 在 Docker 里不同 `docker exec` 会话的 SHM 段互相隔离, 导致节点能发现对方但数据不通. 必须用 XML 配置禁用 SHM:

```bash
# 在容器内写入配置
docker exec 3d_nav_ros2 bash -c 'cat > /tmp/fastrtps_docker.xml << EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles>
  <participant profile_name="default" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp_only</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
  <transport_descriptors>
    <transport_descriptor><transport_id>udp_only</transport_id><type>UDPv4</type></transport_descriptor>
  </transport_descriptors>
</profiles>
EOF'

# 所有 ROS 2 命令前 export 此变量
export FASTRTPS_DEFAULT_PROFILES_FILE=file:///tmp/fastrtps_docker.xml
```

此配置文件在 `config/ros2/fastrtps_docker.xml`.

## 操作指南

### Track 1a: 离线建图 (ROS 1)

建图用 ROS 1 容器, 见 main 分支 README. 产物:
- `scans.pcd` — 3D 全局点云 → 放入 `~/g1_3d_nav/maps/`
- `accumulated_grid.pgm` + `.yaml` — 2D 栅格地图 → 放入 `~/g1_3d_nav/maps/`

### Track 2: ROS 2 导航

#### 前提

1. 容器已运行: `docker start 3d_nav_ros2`
2. 地图文件已放入 `~/g1_3d_nav/maps/`:
   - `scans_0518.pcd` — 3D 点云地图
   - `accumulated_grid.pgm` + `accumulated_grid.yaml` — 2D 栅格
3. DDS 配置已写入 `/tmp/fastrtps_docker.xml`

#### 一键启动

```bash
bash scripts/ros2/start_ros2_nav.sh
```

#### 手动分步启动

**G1 终端 1 — MID360 驱动:**

```bash
docker exec -it 3d_nav_ros2 bash -c '
  export FASTRTPS_DEFAULT_PROFILES_FILE=file:///tmp/fastrtps_docker.xml
  source /opt/ros/humble/setup.bash
  source /root/3d_nav_g1/livox_ws/install/setup.bash
  ros2 launch livox_ros_driver2 msg_MID360_launch.py
'
```

**G1 终端 2 — FAST-LIO:**

```bash
docker exec -it 3d_nav_ros2 bash -c '
  export FASTRTPS_DEFAULT_PROFILES_FILE=file:///tmp/fastrtps_docker.xml
  source /opt/ros/humble/setup.bash
  source /root/3d_nav_g1/g1_ws/install/setup.bash
  ros2 launch fast_lio mapping.launch.py rviz:=false
'
```

> FAST-LIO 配置文件中 `lid_topic` 必须是 `/livox/lidar`, 不是 `/livox/custom_msg`.

**G1 终端 3 — open3d_loc (3D ICP 重定位):**

```bash
docker exec -it 3d_nav_ros2 bash -c '
  export FASTRTPS_DEFAULT_PROFILES_FILE=file:///tmp/fastrtps_docker.xml
  source /opt/ros/humble/setup.bash
  source /root/3d_nav_g1/g1_ws/install/setup.bash
  ros2 launch open3d_loc open3d_loc_g1.launch.py rviz:=false
'
```

> 加载 258MB PCD 约需 30-40 秒. 定位开始时 `map→odom` TF 开始广播, `/localization_3d_confidence` 输出置信度.

**G1 终端 4 — map_server + Nav2:**

```bash
# Map server
docker exec -it 3d_nav_ros2 bash -c '
  export FASTRTPS_DEFAULT_PROFILES_FILE=file:///tmp/fastrtps_docker.xml
  source /opt/ros/humble/setup.bash
  ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:=/root/maps/accumulated_grid.yaml \
    -r __node:=map_server
'

# Nav2
docker exec -it 3d_nav_ros2 bash -c '
  export FASTRTPS_DEFAULT_PROFILES_FILE=file:///tmp/fastrtps_docker.xml
  source /opt/ros/humble/setup.bash
  ros2 launch nav2_bringup navigation_launch.py \
    params_file:=/root/nav2_params.yaml
'
```

**G1 Host — 底盘控制:**

```bash
# 在 G1 宿主机上, 用 Python 3.10 + SDK2
python3 g1_control/cmd_vel_to_sdk2.py
```

#### 发送导航目标

```bash
docker exec 3d_nav_ros2 bash -c '
  export FASTRTPS_DEFAULT_PROFILES_FILE=file:///tmp/fastrtps_docker.xml
  source /opt/ros/humble/setup.bash
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: \"map\"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}"
'
```

## 关键配置变更 (相对于 Nav2 原版参数)

所有 G1 特定改动在 `config/ros2/nav2_params.yaml` 中以 `# G1:` 注释标出. 核心变更:

| 参数 | 原版 | G1 | 原因 |
|------|------|----|------|
| `robot_base_frame` | `base_link` | `body` | G1 TF 树用 `body` 帧 |
| `odom_topic` | `/odom` | `/Odometry_loc` | FAST-LIO 的 topic 名 |
| `controller_plugin` | `dwb_core::DWBLocalPlanner` | `RegulatedPurePursuit` | 双足: 先转再走, 不后退 |
| `max_linear_vel` | 0.26 | 0.4 | G1 最大步行速度 |
| `robot_radius` | 0.22 | 0.3 | G1 更大占地 |
| `use_astar` | false | true | 复杂室内环境 A* 更好 |
| `global_frame` (local) | `odom` | `map` | 统一使用 map 坐标系 |

## 底盘控制

`cmd_vel_to_sdk2.py` 直接订阅 `/cmd_vel_smooth` (Twist), 调用 Unitree SDK2 控制 G1 运动:

```python
robot = LocoClient()
robot.Init()         # 必须! 否则 Move() 不生效
robot.Start()
robot.Move(vx=vx, vy=vy, vyaw=wz, continous_move=True)
```

速度死区: 线速度 < 0.03 m/s 且角速度 < 0.03 rad/s 时调用 `StopMove()`.

## 验证

```bash
# 定位置信度 (> 0.7 锁定, > 0.9 高精度)
docker exec 3d_nav_ros2 bash -c '
  source /opt/ros/humble/setup.bash
  ros2 topic echo /localization_3d_confidence --once
'

# 当前位姿
docker exec 3d_nav_ros2 bash -c '
  source /opt/ros/humble/setup.bash
  ros2 topic echo /localization_3d --once
'

# TF 树
docker exec 3d_nav_ros2 bash -c '
  source /opt/ros/humble/setup.bash
  ros2 run tf2_tools view_frames
'

# 所有节点
docker exec 3d_nav_ros2 bash -c '
  source /opt/ros/humble/setup.bash
  ros2 node list
'
```

## 已知问题

1. **DDS Docker SHM**: FastRTPS 默认 shared memory 在 Docker 里不通, 必须用 `fastrtps_docker.xml` 禁用 SHM.
2. **FAST-LIO topic 错配**: 默认配置订阅 `/livox/custom_msg`, 但 LiDAR 驱动发布 `/livox/lidar`. 需改 `mid360.yaml` 中 `lid_topic`.
3. **open3d_loc 地图路径**: launch 文件中 `map_file` 必须指向实际存在的 PCD 文件.
4. **Kalman filter NaN**: `kf_baselink2map/*` 参数用 `/` 声明可能与 ROS 2 参数系统不兼容, 导致 motion_link TF 出现 NaN. 不影响 `map→odom` TF 和主定位.
5. **WiFi 跨机器**: DDS 跨 WiFi 有丢帧, 建图/导航在 G1 本地进行.

## 相关仓库

- [g1_3d_nav](https://github.com/leokkzzhzzz/g1_3d_nav) — 本项目
- [deepglint FAST_LIO_LOCALIZATION_HUMANOID](https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID) — FAST-LIO + open3d_loc 源码

## 硬件

| 组件 | 型号 | IP |
|------|------|-----|
| 机器人 | Unitree G1 Edu | — |
| 板载计算机 | Jetson Orin NX | 192.168.123.164 (有线) / 192.168.100.30 (WiFi) |
| LiDAR | Livox MID360 (倒装 roll=180°) | 192.168.123.120 |

## 版本

| 版本 | 日期 | 分支 | 内容 |
|------|------|------|------|
| v3.6.0 | 2026-05-19 | ros2 | ROS 2 Nav2 + Regulated Pure Pursuit, DDS Docker 修复 |
| v3.5.0 | 2026-05-18 | main | ROS 1 完整导航部署文档 + GCR 镜像 |
| v3.4.0 | 2026-05-15 | main | move_base + DWA + velocity_smoother + bridge + SDK2 |
| v3.0.0 | 2026-05-14 | main | ROS 1 主方案 — open3d_loc + RViz + 定位 |
