# PRD: Track 1a — G1 离线 3D LiDAR 建图

**Status**: In Progress (建图容器 + 驱动已就绪，待完整移动建图)
**Created**: 2026-05-12
**Context**: [g1_3d_nav](https://github.com/leokkzzhzzz/g1_3d_nav)

---

## Problem Statement

Unitree G1 人形机器人需要生成高质量 3D 点云地图（PCD），作为后续 3D OctoMap 导航系统的输入。G1 的 Livox MID360 LiDAR 倒装安装（roll=180°），标准 livox_ros_driver2 只旋转点云外参、不旋转 IMU 外参，导致 IMU-LiDAR 数据方向矛盾，FAST-LIO2 建图发散。需要一套能在 G1 实机 Jetson Orin NX 上正确运行的离线建图方案。

## Solution

使用 deepglint 团队为 G1 定制的修改版 livox_ros_driver2（同时旋转点云和 IMU 外参）+ FAST-LIO2 建图节点，运行在 ROS 1 Noetic Docker 容器中。容器通过 `--network host` 访问 G1 内部网络的 MID360（192.168.123.120），X server 授权后支持 G1 本地 RViz 显示。Leo 笔记本通过 WiFi 运行独立 RViz 容器远程连接 G1 ROS master 进行实时建图可视化。

### 架构

```
G1 Jetson Orin NX                          Leo 笔记本
┌──────────────────────────────┐          ┌─────────────────────┐
│ hongtu_mapper 容器            │  WiFi    │ g1_rviz 容器         │
│ (ROS 1 Noetic)               │ ◄────── │ (ROS 1 Noetic)       │
│                              │  11311   │ ROS_MASTER_URI → G1 │
│ deepglint livox_ros_driver2  │          │ rviz (官方配置)      │
│   → 点云+IMU 同时 roll=180   │          └─────────────────────┘
│   → /livox/lidar CustomMsg   │
│   → /livox/imu               │
│                              │
│ fast_lio_mapping             │
│   → /cloud_registered_body   │
│   → /Laser_map_1             │
│   → /slam_odom               │
│   → TF (body)                │
│                              │
│ 一键启动:                     │
│ mapping_g1_full.launch       │
└──────────────────────────────┘
```

## User Stories

1. 作为建图操作者，我想要一条命令启动驱动+FAST-LIO（不含 RViz），避免 Jetson 卡死
2. 作为建图操作者，我想在自己的笔记本上通过 RViz 远程实时查看建图点云，不消耗 G1 算力
3. 作为建图操作者，我想要 MID360 倒装（roll=180°）的情况下 IMU 和点云数据方向一致，FAST-LIO2 不发散
4. 作为建图操作者，我想要单核编译（-j1）避免 Jetson Orin NX 8GB 内存 OOM
5. 作为建图操作者，我想要地图保存为 PCD 文件到宿主机挂载目录，供后续重定位和 OctoMap 转换使用
6. 作为开发者，我想要 Docker 镜像可复现，后续可快速重建环境

## Implementation Decisions

### 驱动选型：deepglint 替代 HongTu

**Decision**: 建图使用 deepglint 修改版 livox_ros_driver2，替代 HongTu 的驱动。

**Why**: HongTu 的标准 livox_ros_driver2 的 `extrinsic_parameter.roll` 只旋转点云坐标、不旋转 IMU 数据。G1 的 MID360 倒挂安装导致 IMU 重力方向反了，FAST-LIO2 移动时必然发散。deepglint 修改版同时旋转点云和 IMU 外参，README 明确说明"Livox MID360 on Unitree G1 is upside down, and the stock driver only allows pointcloud extrinsics changes, not IMU"。

### 包名不匹配修复

**Decision**: 修改 deepglint FAST_LIO 源码中所有 `livox_ros_driver` 引用为 `livox_ros_driver2`（CMakeLists.txt 依赖、package.xml 依赖、C++ include 路径、命名空间前缀）。

**Why**: deepglint 仓库中驱动包名为 `livox_ros_driver2`，但 FAST_LIO 子模块的依赖和源码引用的是 `livox_ros_driver`（旧包名约定）。需统一包名才能通过 cmake 配置。

### 编译策略

**Decision**: 使用 `catkin_make -DROS_EDITION=ROS1 -j1` 单核编译。编译前需先 `make fast_lio_generate_messages_cpp` 生成自定义消息头文件。

**Why**: Jetson Orin NX 8GB 内存，4 核并行编译 C++ 大项目（PCL + GTSAM 依赖）会导致 OOM kill。消息生成必须在编译前执行，否则报 `fatal error: fast_lio/Pose6D.h: No such file or directory`。

### 外参修正

**Decision**: `lidar2base_rpy` 从 HongTu 默认的 `[0, 0.45, 0]`（25.8° pitch）改为 `[0, 0, 0]`。

**Why**: 0.45 rad 是 HongTu 自身机器人的值，G1 头部 MID360 大致水平。静止测试验证：改前 PCD 91% 为垃圾数据（45km 跨度），改后 100% 有效点（0 离群点，Z 范围 1.8m 合理）。

### RViz 远程可视化

**Decision**: G1 上建图不启动本地 RViz（`rviz:=false`）。Leo 笔记本通过独立 ROS 1 Docker 容器设 `ROS_MASTER_URI=http://<G1_IP>:11311` 远程运行 RViz。

**Why**: Jetson Orin NX 同时渲染 3D 点云和跑 SLAM 导致严重卡顿。远程 RViz 将渲染负载转移到笔记本 GPU，G1 专注建图计算。

### 一键启动

**Decision**: 创建 `mapping_g1_full.launch`，include 驱动 + FAST-LIO 两个 launch 文件。RViz 默认 `true`（通过 `mapping_mid360_g1.launch` 的 `rviz` arg 控制），G1 本机启动时传 `rviz:=false`。

**Why**: 避免分三次手动启动（驱动 → FAST-LIO → RViz），减少操作错误。

### 网络配置

| 设备 | IP | 端口 |
|------|-----|------|
| MID360 LiDAR | 192.168.123.120 | 56100-56500 |
| G1 Jetson（有线） | 192.168.123.164 | — |
| G1 Jetson（WiFi） | 192.168.100.30 | — |
| ROS Master | 0.0.0.0（容器 --network host）| 11311 |

### Docker 镜像

**Image**: `hongtu-fastlio2:noetic`（5.7GB）
- Base: `ros:noetic-ros-base` (ARM64)
- 额外包: libtbb-dev, libpcl-dev, libeigen3-dev, ros-noetic-gtsam
- Livox-SDK2 编译到 /usr/local
- deepglint workspace: `/root/g1_3d_nav/deepglint_ws`

## Testing Decisions

### 测试原则
- 只测试外部可观测行为（PCD 点云质量、话题发布、IMU 方向），不测试 FAST-LIO2 内部算法
- 测试通过标准：静止 PCD 无离群点 + 移动 PCD 与场景几何一致

### 已完成验证

| 测试项 | 方法 | 结果 |
|--------|------|------|
| MID360 连接 | `ping 192.168.123.120` | <1ms 延迟，连通 |
| 驱动启动 | `rostopic list \| grep livox` | /livox/lidar + /livox/imu 正常发布 |
| FAST-LIO 启动 | 启动日志 | IMU Initial Done |
| 静止点云质量 | 保存 PCD，分析 bounding box | 0 离群点，Z 0.4-1.4m 合理 |
| 编译产物 | `ls devel/lib/fast_lio/` | fastlio_mapping 可执行文件 |
| 远程 RViz | `rostopic list` 从 Leo 笔记本 | 话题列表正确获取 |

### 待验证
- [ ] 移动建图：遥控 G1 走一圈，PCD 结构匹配真实场景
- [ ] GTSAM 回环：走闭环路径，点云自动对齐

## Out of Scope

- 回环检测优化（GTSAM 参数调优）
- 在线 SLAM（建图阶段不要求实时性）
- 多楼层合并建图
- PCD → OctoMap 转换（Track 2）
- ROS 2 运行时定位（Track 1b）
- G1 运动控制（Track 3）

## Further Notes

### 已知问题

1. Docker bridge 网络构建失败（Jetson iptables raw 表缺失），需用手动 `docker run --network host` + `apt-get install` + `docker commit` 方式构建镜像
2. 容器内 `ping` 和 `rviz` 需额外安装（非基础镜像包含）
3. `mapping.launch` 原包含 octomap_server 引用但包未安装，已精简为纯建图

### 后续 Track 依赖关系

```
Track 1a (本 PRD) ──→ map.pcd
                          │
          ┌───────────────┼───────────────┐
          ▼                               ▼
    Track 1b (重定位)               Track 2 (OctoMap)
    deepglint open3d_loc            jie_3d_nav 导入+规划+Web
          │                               │
          └───────────────┬───────────────┘
                          ▼
                    Track 3 (g1pilot 控制器)
```

### 仓库

GitHub: https://github.com/leokkzzhzzz/g1_3d_nav (private, v1.0.3)
