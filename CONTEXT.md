# G1 3D Navigation Deployment Context

This file is the domain language and architectural decisions for deploying `jie_3d_nav` 3D OctoMap navigation on the Unitree G1 humanoid robot.

## Execution Brief

- **Target**: Unitree G1 Edu (Jetson Orin NX, Livox MID360, Intel D435i)
- **ROS 2**: Humble, Docker container (runtime)
- **ROS 1**: Noetic, Docker container (offline mapping only)
- **Goal**: Replace 2D Nav2 navigation with 3D OctoMap-based multi-floor navigation (elevator transitions)

## Language

**3D OctoMap 导航**:
用 OctoMap 体素地图代替 2D 占据栅格进行路径规划。规划器在 3D 体素空间运行 A*，输出带 z 轴的 waypoint 序列。
_Avoid_: 2D Nav2 导航, 2D 代价地图

**离线建图**:
在 ROS 1 独立容器中运行 HongTu FAST-LIO2（含 GTSAM 回环），一次性扫描环境并产出 PCD 地图文件。产出后容器不再运行。
_Avoid_: 在线建图, 实时 SLAM 建图, 运行时边扫边建

**全局重定位**:
启动时通过当前 LiDAR 扫描与离线建图产出的 PCD 地图做 ICP 匹配，恢复机器人在全局 OctoMap 中的位姿。
_Avoid_: 固定启动位, Odin 1 绝对定位, 纯里程计推演

**离线地图包**:
预建的 OctoMap 地图（PCD → pcd_to_octomap_node → map_package_manager 保存），包含 meta.yaml + octomap_msg.npz + layers.npz。
_Avoid_: 单独 .bt/.ot 文件, 2D PGM/YAML 地图

**G1 运动控制**:
通过 Unitree SDK `LocoClient::SetVelocity(vx, 0, omega)` 控制 G1 行走。G1 不支持侧移（vy=0），需先走 FSM 状态链（Damp → StandUp → Start(500) → BalanceStand）。
_Avoid_: D1 侧移控制, d1_controller 的四足 offset 补偿

**MID360 倒装补偿**:
G1 头部 MID360 倒挂安装。标准 livox_ros_driver2 只旋转点云不旋转 IMU，需用 deepglint 修改版 driver 同时旋转点云和 IMU 外参（roll=180）。
_Avoid_: 只调点云外参, 用 imu_ext_rot 补偿 IMU 方向

**Map Package**:
jie_3d_nav 的地图保存格式，一个目录包含三个文件。存储完整 OctoMap + 规划参数 + 编辑层数据。
_Avoid_: 单独 .bt/.ot 文件, 2D PGM/YAML 地图

**双容器架构**:
ROS 1 容器只做离线建图（HongTu），ROS 2 容器做全部运行时（定位 + OctoMap + 规划 + 控制）。两容器之间唯一接口是 PCD 文件。
_Avoid_: ROS1/ROS2 bridge 运行时耦合, 混合工作区

## Architecture (Current)

```
┌─ ROS 1 容器 (离线建图, 一次性) ───────────┐
│ deepglint livox_ros_driver2 (roll:180)    │
│ fast_lio_mapping                          │
│   → 产出 scans.pcd (129MB, 419万点)       │
│ pcd_save_en: true, interval: -1           │
│   → Ctrl-C 自动保存                       │
└───────────────────────────────────────────┘
              │
              ▼  scans.pcd
┌─ ROS 2 容器 (运行时: 3d_nav_g1) ──────────┐
│ ✅ livox_ros_driver2 (驱动, HUMBLE_ROS)    │
│ ✅ fast_lio (FAST-LIO2 odometry)          │
│ ✅ open3d_loc (ICP 全局重定位)             │
│        │                                   │
│        ├── /localization_3d                │
│        ├── TF map→odom→base_footprint      │
│        │                                   │
│ ⬜ octomap_server → /octomap   (Track 2)  │
│ ⬜ jie_3d_nav + Web UI        (Track 2)  │
│ ⬜ g1pilot 控制               (Track 3)  │
└───────────────────────────────────────────┘

## Architecture Decisions

### ADR-001: 离线建图 — deepglint ROS 1 独立容器

使用 deepglint FAST_LIO_LOCALIZATION_HUMANOID（ROS 1 main 分支）在独立容器中完成一次性离线建图，产出 scans.pcd。
**Why**: deepglint 修改版 livox_ros_driver2 同时旋转点云和 IMU 外参（roll=180°），解决了 G1 MID360 倒装问题。pcd_save_en:true + interval:-1 实现 Ctrl-C 自动保存全部点云。
**How to apply**: G1 上启动 ROS 1 容器 → roslaunch fast_lio mapping_g1_full.launch rviz:=false → 遥控扫描 → Ctrl-C 自动保存 scans.pcd。

### ADR-002: 运行时定位 — deepglint ROS 2 Humble 容器

选择 deepglint 的 Humble 分支，在独立 ROS 2 容器 `3d_nav_g1` 中编译三件套。
**Why**: 与后续 Track 2 (jie_3d_nav) 和 Track 3 (g1pilot) 统一 ROS 2 Humble 环境，避免 ROS1/ROS2 桥接。open3d_loc 通过 ICP 将当前扫描匹配到离线 PCD，实现全局重定位。
**How to apply**: 容器内置 `/root/3d_nav_g1/start.sh` 一键启动。需先启动 livox 驱动（start_driver.sh），再启动定位。

### ADR-003: 地图组织 — 单一大 OctoMap

所有楼层合并为一个 OctoMap，不按楼层拆分 map package。
**Why**: jie_3d_nav 原生支持单 OctoMap 规划，避免运行时切换地图。电梯过渡由外部状态机处理，不在规划层体现。
**How to apply**: 各楼层 PCD 合并后通过 pcd_to_octomap_node 导入，保存为单个 map package。

### ADR-004: 控制器 — g1pilot 风格 G1 Controller

使用 g1pilot 的 waypoint 跟踪逻辑 + Unitree LocoClient，不修改 d1_controller。
**Why**: d1_controller 包含 D1 四足 offset、侧移逻辑，与 G1 双足平台不兼容。g1pilot 已在 G1 上验证过 nav2point 路径跟踪。
**How to apply**: 参考 nav2point.py 的 waypoint 跟踪和 loco_client 的 Unitree SDK 接口，对接 jie_3d_nav 的 /planned_path + /start_navigation + /stop_navigation。

### ADR-005: 工作空间组织 — G1 实机独立 ROS 2 工作区 + 独立 ROS 1 容器

jie_3d_nav 和 deepglint 在 G1 实机构建独立 ROS 2 colcon workspace。HongTu 建图在独立 ROS 1 容器中运行。
**Why**: rl_hnav 是 5080 仿真机环境，与实机 G1 无关。ROS 1 离线建图和 ROS 2 运行时完全隔离，通过 PCD 文件传递数据。
**How to apply**: ROS 2 侧在 G1 上创建 `~/g1_3d_ws/src/`，clone 所有 ROS 2 包。ROS 1 侧拉取 HongTu 镜像或 Dockerfile，挂载宿主机目录保存 PCD 产物。

### ADR-006: 跨机器数据验证 — ros1_bridge 桥接方案

ROS2 DDS 在多网卡（WiFi + Tailscale + Docker）跨机器环境下 topic 数据不稳定。短期验证方案：在 G1 宿主机运行 `ros1_bridge`，将 ROS2 topic 映射到 ROS1 roscore，Leo 通过 ROS1 TCP 链路查看数据。
**Why**: Zenoh bridge 和 TCP network_bridge 两种方案都因 Leo 端 DDS 多网卡摇摆导致数据无法稳定到达。ROS1 roscore 的纯 TCP 架构已证明在 Tailscale/跨子网下可靠。ros1_bridge 在 G1 本地同时接入 ROS2 DDS 和 ROS1 roscore（均在 localhost），避开跨机器 DDS。
**How to apply**: 使用 [ros-humble-ros1-bridge-builder](https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder) 在 G1 上构建预编译的 `ros1_bridge` 包，启动时运行 `dynamic_bridge --bridge-all-2to1-topics`。详见 `docs/ROS1_BRIDGE_SOLUTION.md`。

## Relationships

- **离线建图** 先于 **全局重定位**：必须先产出一份高质量的 PCD 地图
- **全局重定位** 先于 **3D OctoMap 导航**：启动时必须先确定机器人在 OctoMap 中的位姿
- **离线地图包** 由 PCD 通过 pcd_to_octomap_node 转换产生
- **离线建图** 和 **离线地图包** 可以独立更新：重新扫图后替换 PCD 和 OctoMap
- **G1 运动控制** 不执行侧移，所有路径跟踪的 vy 分量必须为 0
- 离线建图时 ROS 1 和 ROS 2 之间唯一的接口是 **PCD 文件**
- **ros1_bridge** 运行时桥接 ROS2 → ROS1，仅用于验证，非生产路径

## Deployment Tracks

| Track | Scope | Runtime | Status |
|-------|-------|---------|--------|
| Track 1a | deepglint ROS 1 离线建图 → scans.pcd | ROS 1 容器 `hongtu-fastlio2:noetic` | ✅ 完成 |
| Track 1b | deepglint ROS 2 重定位 | ROS 2 容器 `3d_nav_g1` | ✅ 编译完成, ⬜ 冒烟测试 |
| Track 1c | ros1_bridge 跨机器数据验证 | G1 宿主机 | ⬜ 待构建 |
| Track 2 | jie_3d_nav OctoMap + 规划 + Web | ROS 2 容器 `3d_nav_g1` | ⬜ 未开始 |
| Track 3 | g1pilot 控制器接入 | ROS 2 容器 `3d_nav_g1` | ⬜ 未开始 |
| Track 4 | ROS2 DDS 直连（多网卡修复后） | G1 ↔ Leo 原生 ROS2 | ⬜ 未开始 |

Track 1a → 产出 scans.pcd → Track 1b + Track 2 并行 → Track 3。
Track 1c 为验证 Track 1b 数据正确性的中间方案，Track 4 为最终目标。
