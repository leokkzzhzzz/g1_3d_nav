# 深度研究报告：Track 2 & Track 3 技术分析

**项目**: G1 3D Navigation (`leokkzzhzzz/g1_3d_nav`)
**日期**: 2026-05-12
**版本**: v1.0
**作者**: Kiro AI Deep Research

---

## 目录

1. [研究概述](#1-研究概述)
2. [Track 2: jie_3d_nav 3D OctoMap 导航系统](#2-track-2-jie_3d_nav-3d-octomap-导航系统)
   - 2.1 [系统架构总览](#21-系统架构总览)
   - 2.2 [包结构与依赖](#22-包结构与依赖)
   - 2.3 [PCD → OctoMap 转换引擎](#23-pcd--octomap-转换引擎)
   - 2.4 [3D A* 路径规划器 (jie_path_node)](#24-3d-a-路径规划器-jie_path_node)
   - 2.5 [地图包管理系统](#25-地图包管理系统)
   - 2.6 [Web 3D 可视化与交互](#26-web-3d-可视化与交互)
   - 2.7 [d1_controller 路径跟踪控制器](#27-d1_controller-路径跟踪控制器)
   - 2.8 [G1 适配要点与改造清单](#28-g1-适配要点与改造清单)
3. [Track 3: g1pilot 控制器接入](#3-track-3-g1pilot-控制器接入)
   - 3.1 [g1pilot 系统架构](#31-g1pilot-系统架构)
   - 3.2 [Unitree SDK2 LocoClient API 深度分析](#32-unitree-sdk2-lococlient-api-深度分析)
   - 3.3 [G1LocoClient ROS 2 节点](#33-g1lococlient-ros-2-节点)
   - 3.4 [nav2point 路径跟踪器](#34-nav2point-路径跟踪器)
   - 3.5 [dijkstra_planner 2D 规划器](#35-dijkstra_planner-2d-规划器)
   - 3.6 [RobotState 状态发布器](#36-robotstate-状态发布器)
   - 3.7 [接口对接方案](#37-接口对接方案)
4. [集成架构与话题拓扑](#4-集成架构与话题拓扑)
5. [关键技术风险与缓解策略](#5-关键技术风险与缓解策略)
6. [实施路线图](#6-实施路线图)
7. [参考文献](#7-参考文献)

---

## 1. 研究概述

本报告对 G1 3D Navigation 项目中 **Track 2**（jie_3d_nav 3D OctoMap 导航）和 **Track 3**（g1pilot 控制器接入）进行源码级深度分析，覆盖：

- 上游仓库 [`6-robot/jie_3d_nav`](https://github.com/6-robot/jie_3d_nav) 全部 3 个 ROS 2 包的完整源码
- Leo 的测试 fork [`leokkzzhzzz/jie_3d_nav_test`](https://github.com/leokkzzhzzz/jie_3d_nav_test)
- [`hucebot/g1pilot`](https://github.com/hucebot/g1pilot) 导航+控制模块
- [`unitreerobotics/unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python) LocoClient 底层 API
- G1 实机硬件约束（Jetson Orin NX 8GB, MID360 倒装, 无侧移）

**核心发现**: jie_3d_nav 输出 `/planned_path` (nav_msgs/Path)，g1pilot 的 nav2point 消费 Path 消息并输出 Joy 虚拟摇杆，loco_client 将 Joy 转为 `LocoClient.Move(vx, vy, vyaw)` 调用。两者通过 **3 个 ROS 2 话题**即可对接，无需修改任何上游核心代码。

---


## 2. Track 2: jie_3d_nav 3D OctoMap 导航系统

### 2.1 系统架构总览

jie_3d_nav 是由 [6-robot](https://github.com/6-robot) 开源的 ROS 2 Humble 三维导航系统，原始设计面向智元科技 D1 机器狗 + 留形科技 Odin 1 空间定位模组。系统通过 OctoMap 八叉树体素地图实现三维空间的路径规划。

**源码引用**: [`6-robot/jie_3d_nav` README.md](https://github.com/6-robot/jie_3d_nav/blob/main/README.md)

核心数据流：

```
PCD 文件 → pcd_to_octomap_node → /octomap (OctoMap msg)
                                      ↓
                              jie_path_node (3D A*)
                                      ↓
                              /planned_path (nav_msgs/Path)
                                      ↓
                              d1_controller → /cmd_vel (Twist)
```

### 2.2 包结构与依赖

| 包名 | 类型 | 核心产物 | 引用 |
|------|------|---------|------|
| `jie_map_msgs` | 接口包 | 4 个 srv 定义 | `jie_map_msgs/srv/` |
| `jie_octomap` | C++/Python 混合 | 8 个 C++ 节点 + 6 个 Python 脚本 + Web UI | `jie_octomap/src/`, `jie_octomap/scripts/`, `jie_octomap/web/` |
| `octo_planner` | C++ | jie_path_node + d1_controller + test_tf_node | `octo_planner/src/` |

**服务接口定义** (源码: `jie_map_msgs/srv/`):

```
# LoadNavigationMapPackage.srv
string package_path
---
bool success
string message
string map_id

# SaveNavigationMapPackage.srv
string package_path
bool overwrite
---
bool success
string message
string manifest_path

# GetNavigationMapMeta.srv (由 jie_path_node 提供)
# ExportNavigationSnapshot.srv (由 jie_path_node 提供)
```

**系统依赖** (源码: `install_deps_humble.sh`):

| 依赖 | 用途 | 安装方式 |
|------|------|---------|
| OctoMap / octomap_msgs | 八叉树核心库 | `ros-humble-octomap` |
| Open3D (C++) | PCD 读写、体素下采样 | 源码编译或 `libopen3d-dev` |
| OpenCV | d1_controller 调试视图 | `libopencv-dev` |
| rosbridge_server | Web UI WebSocket 通信 | `ros-humble-rosbridge-server` |
| PyQt5 / VTK | GUI 地图编辑器 | `python3-pyqt5`, `python3-vtk9` |

> **G1 注意**: Open3D C++ 在 ARM64 (Jetson Orin NX) 上需源码编译，apt 无预编译包。这是 Track 2 最大的编译风险点。

### 2.3 PCD → OctoMap 转换引擎

**源码**: `jie_octomap/src/pcd_to_octomap_node.cpp` (226 行)

#### 算法流程

```
1. Open3D::ReadPointCloud(pcd_file)
2. [可选] VoxelDownSample(voxel_downsample_m)
3. 逐点 → OcTreeKey → 统计每个体素内点数
4. 过滤: 点数 < min_points_per_voxel 的体素丢弃
5. BFS 聚类: 体素数 < min_cluster_voxels 的孤立簇丢弃
6. updateNode(coord, occupied=true) 插入 OcTree
7. updateInnerOccupancy() 更新内部节点
8. 发布 /octomap (transient_local, reliable)
```

#### 关键参数

| 参数 | 默认值 | 说明 | G1 建议值 |
|------|--------|------|-----------|
| `resolution` | 0.2 | OctoMap 分辨率 (m) | 0.1~0.15 (室内精细导航) |
| `voxel_downsample_m` | 0.0 | 预下采样粒度 | 0.05 (减少 PCD 体积) |
| `min_points_per_voxel` | 3 | 体素内最少点数 | 3~5 |
| `min_cluster_voxels` | 4 | 最小连通簇体素数 | 4~8 (去噪) |
| `frame_id` | "map" | OctoMap 坐标帧 | "map" |

#### 发布机制

- QoS: `transient_local` + `reliable`，确保后启动的节点也能收到最新地图
- 定时器每 1 秒重发一次（保持 late-joiner 可用）
- 支持 `/pcd_file_cmd` 话题动态切换 PCD 文件

**引用**: `pcd_to_octomap_node.cpp:L40-L60` (参数声明), `L85-L165` (loadPcd 核心逻辑)

### 2.4 3D A* 路径规划器 (jie_path_node)

**源码**: `octo_planner/src/jie_path_node.cpp` (1177 行，最核心的规划模块)

#### 数据结构

```cpp
struct GridIndex { int x, y, z; };  // 离散体素坐标
struct QueueNode { GridIndex idx; double f; double g; };  // A* 优先队列元素
```

#### 规划算法详解

1. **世界坐标 → 网格坐标转换**:
   ```cpp
   GridIndex worldToGrid(x, y, z) {
     return { floor(x/resolution), floor(y/resolution), floor(z/resolution) };
   }
   ```

2. **起终点捕捉** (`findNearestFreeCell`):
   - 用户点击可能在 occupied 或 preblocked 体素内
   - 球形搜索 `snap_search_radius_cells` 范围内最近的 traversable 体素
   - 搜索半径: 默认 8 cells = 1.6m (0.2m 分辨率)

3. **可通行性判定** (`isCellTraversable`):
   - 要求地面支撑 (`require_ground_support`): 下方必须有 occupied 体素
   - 碰撞检测: 以 `robot_radius` 为半径做体积碰撞检查
   - preblocked 回避: 不穿越自动标记的禁行区域
   - 支持严格/宽松两种地面检测模式

4. **A* 搜索**:
   - **26 连通方向** (含对角线)
   - 启发函数: 欧几里得距离
   - 代价函数: `g + step_cost + preblocked_costmap_weight × proximity_cost`
   - 最大迭代: 250,000~500,000
   - 输出: `nav_msgs/msg/Path` + 可视化 LINE_STRIP Marker

5. **Preblocked 代价地图**:
   - 自动识别悬崖边缘、天花板下方等危险区域
   - 向周围 `preblocked_costmap_radius_cells` 扩散渐变代价
   - 权重: `preblocked_costmap_weight` (默认 1.5~2.5)

#### 关键话题接口

| 话题 | 方向 | 类型 | 用途 |
|------|------|------|------|
| `/octomap` | 订阅 | `octomap_msgs/msg/Octomap` | 输入地图 |
| `/start_point` | 订阅 | `geometry_msgs/msg/PointStamped` | 起点 |
| `/goal_point` | 订阅 | `geometry_msgs/msg/PointStamped` | 终点 |
| `/goal_pose` | 订阅 | `geometry_msgs/msg/PoseStamped` | 终点+朝向 |
| `/planned_path` | 发布 | `nav_msgs/msg/Path` | **输出路径** |
| `/planned_path_marker` | 发布 | `visualization_msgs/msg/Marker` | RViz 可视化 |
| `/traversable_cells_markers` | 发布 | `Marker` | 可通行区域 |
| `/preblocked_cells_markers` | 发布 | `Marker` | 禁行区域 |
| `/risk_cost_cells` | 发布 | `PointCloud2` | 风险代价云 |

**引用**: `jie_path_node.cpp:L86-L140` (参数声明), `L850-L1050` (A* planAndPublish 核心)

### 2.5 地图包管理系统

**源码**: `jie_octomap/scripts/map_package_manager` (Python 节点)

地图包格式 (`Map Package`):
```
~/maps/B1/
├── meta.yaml            # 元数据（分辨率、帧ID、来源）
├── octomap_msg.npz      # 序列化 OctoMap 二进制
└── layers.npz           # 编辑层（preblocked、traversable、occupied 修改）
```

服务接口:
- `/map_package_manager/load_navigation_map_package` → 加载并发布 `/octomap`
- `/map_package_manager/save_navigation_map_package` → 保存当前地图状态

### 2.6 Web 3D 可视化与交互

**源码**: `jie_octomap/web/main.js` (1636 行 Three.js 应用)

#### 技术栈

| 组件 | 技术 | 版本 |
|------|------|------|
| 3D 渲染 | Three.js (ES Module) | 本地 vendor 目录 |
| ROS 通信 | roslibjs via rosbridge_websocket | WebSocket 9090 端口 |
| HTTP 服务 | `no_cache_http_server.py` (Python) | 8080 端口 |
| 控制 | 虚拟摇杆 + 旋转滑杆 | 纯 JS 手势事件 |

#### 功能矩阵

| 功能 | 实现方式 | 涉及话题 |
|------|---------|---------|
| OctoMap 体素渲染 | InstancedMesh (CUBE_LIST) | `/octomap_occupied_markers` |
| 可通行区域渲染 | InstancedMesh (半透明绿色) | `/traversable_cells_markers` |
| 禁行区域渲染 | InstancedMesh (蓝色) | `/preblocked_cells_markers` |
| 风险代价渲染 | PointCloud2 解析 → 单独 Box | `/risk_cost_cells` |
| 路径渲染 | TubeGeometry + CatmullRomCurve3 | `/planned_path` |
| 起点设置 | 点击 traversable 体素 | 发布 `/start_point` |
| 终点+朝向设置 | 拖拽方向箭头 | 发布 `/goal_point` + `/goal_pose` |
| 导航确认弹窗 | 收到 path 后 2 秒显示 | 发布 `/start_navigation` Bool |
| 紧急停止 | 按钮 | 发布 `/stop_navigation` Bool |
| 手动遥控 | 虚拟摇杆 + 旋转滑杆 | 发布 `/web_cmd_vel` Twist |
| 机器人位姿 | TF 监听 | `/tf`, `/tf_static` |
| 重定位状态 | UI 指示器 | 监听 TF 可用性 |

**引用**: `web/main.js:L1-L50` (DOM 绑定), `L700-L850` (setPath + 导航确认逻辑)

#### 导航交互流程

```
用户点击体素 → /goal_point + /goal_pose
     ↓
jie_path_node 收到 goal → A* 规划 → 发布 /planned_path
     ↓
Web UI 收到 path → 渲染紫色管道 → 2秒后弹出确认框
     ↓
用户点击"开始导航" → /start_navigation (Bool=true)
     ↓
d1_controller 收到 start_navigation → 激活路径跟踪 → /cmd_vel
```

### 2.7 d1_controller 路径跟踪控制器

**源码**: `octo_planner/src/d1_controller.cpp` (908 行)

#### 控制算法

采用 **纯追踪 (Pure Pursuit) 变体** + 终点朝向对齐：

1. **目标点选择**: 当前 tracking point 到达 `tracking_point_reached_xy_tolerance` 后前进
2. **速度计算** (机体坐标系):
   ```
   heading_error = atan2(target_base_y, max(0.001, target_base_x))
   cmd_vel.linear.x = clamp(target_base_x × linear_gain, -max_linear, max_linear)
   cmd_vel.linear.y = clamp(target_base_y × lateral_gain, -max_lateral, max_lateral)
   cmd_vel.angular.z = clamp(heading_error × heading_gain + target_base_y × cross_track_gain, -max_angular, max_angular)
   ```
3. **终点朝向对齐** (`align_final_yaw`): 最后一个 waypoint 到达后，原地旋转至 goal_pose 朝向
4. **死区**: linear/lateral/angular 各有独立死区参数
5. **OpenCV 调试视图**: 实时显示机体坐标系下的路径点和 tracking target

#### 关键话题

| 话题 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `/planned_path` | 订阅 | `nav_msgs/msg/Path` | 输入路径 |
| `/start_navigation` | 订阅 | `std_msgs/msg/Bool` | 确认执行 |
| `/stop_navigation` | 订阅 | `std_msgs/msg/Bool` | 紧急停止 |
| `/web_cmd_vel` | 订阅 | `geometry_msgs/msg/Twist` | 手动遥控 |
| `/cmd_vel` | 发布 | `geometry_msgs/msg/Twist` | **输出速度** |
| `/tracking_point_marker` | 发布 | `Marker` | 当前追踪点 |

#### D1 特有逻辑（G1 不适用）

| 特性 | D1 行为 | G1 需要 |
|------|---------|---------|
| `enable_lateral_motion` | true (四足可侧移) | **false** (人形不支持) |
| `robot_center_offset` | odin1_base_link -0.18m | 不需要 |
| `base_frame` | odin1_base_link | base_footprint |
| OpenCV debug view | 有 | 可保留但 Jetson 无 GUI |

**引用**: `d1_controller.cpp:L210-L290` (控制参数), `L390-L460` (速度计算核心)

### 2.8 G1 适配要点与改造清单

| # | 改造项 | 原因 | 实施方式 |
|---|--------|------|---------|
| 1 | **禁用侧移** | G1 双足不支持 vy | `enable_lateral_motion: false` |
| 2 | **不使用 d1_controller** | 改用 g1pilot nav2point | 不启动 d1_controller，见 Track 3 |
| 3 | **base_frame 改名** | G1 用 `base_footprint` | web_test.launch.py 参数覆盖 |
| 4 | **Open3D ARM64 编译** | Jetson 无 apt 预编译 | 源码 `cmake -DBUILD_CUDA_MODULE=OFF` |
| 5 | **OctoMap 分辨率调优** | 0.2m 太粗 for 室内 | 0.1m，配合 crop 限制内存 |
| 6 | **odin1 依赖剥离** | G1 不用 Odin 1 | 用 `web_test.launch.py` (已解耦) |
| 7 | **TF 帧对齐** | deepglint 定位帧 vs jie_3d_nav | 确保 `/localization_3d` 发布 `map→odom→base_footprint` |

---


## 3. Track 3: g1pilot 控制器接入

### 3.1 g1pilot 系统架构

**仓库**: [`hucebot/g1pilot`](https://github.com/hucebot/g1pilot) (BSD-3-Clause, ROS 2 Jazzy/Humble)
**维护者**: Clemente Donoso, INRIA (clemente.donoso@inria.fr)

g1pilot 是专为 Unitree G1 人形机器人设计的 ROS 2 包，采用"下半身交给 Unitree 内置控制器，上半身自定义控制"的双层架构。

**模块划分**:

```
g1pilot/
├── navigation/          ← Track 3 核心
│   ├── loco_client.py       # Unitree LocoClient ROS 2 封装
│   ├── nav2point.py         # 路径跟踪 → 虚拟 Joy
│   ├── dijkstra_planner.py  # 2D 路径规划 (备用)
│   ├── create_map.py        # 2D 占据栅格创建
│   └── fix_mola_odometry.py # MOLA 里程计帧修复
├── manipulation/        ← 上半身控制 (Track 3 不涉及)
│   ├── opensot_solver.py
│   ├── interactive_marker.py
│   └── dx3_hand.py
├── state/               ← 机器人状态
│   ├── robot_state.py       # 关节/IMU/电机状态发布
│   ├── lights.py
│   └── voice.py
├── teleoperation/       ← 遥操作
│   ├── joystick.py
│   ├── joy_mux.py
│   └── ui_interface.py
└── utils/
```

**引用**: `g1pilot/README.md` (节点列表), `g1pilot/g1pilot/navigation/` (源码目录)

### 3.2 Unitree SDK2 LocoClient API 深度分析

**仓库**: [`unitreerobotics/unitree_sdk2_python`](https://github.com/unitreerobotics/unitree_sdk2_python)
**通信协议**: CycloneDDS RPC (UDP)
**服务名**: `"sport"`
**API 版本**: `"1.0.0.0"`

#### API ID 清单

| API ID | 方向 | 函数名 | 参数 |
|--------|------|--------|------|
| 7001 | GET | `GetFsmId` | — |
| 7002 | GET | `GetFsmMode` | — |
| 7003 | GET | `GetBalanceMode` | — |
| 7004 | GET | `GetSwingHeight` | — |
| 7005 | GET | `GetStandHeight` | — |
| 7101 | SET | `SetFsmId` | `{"data": int}` |
| 7102 | SET | `SetBalanceMode` | `{"data": int}` |
| 7103 | SET | `SetSwingHeight` | `{"data": float}` |
| 7104 | SET | `SetStandHeight` | `{"data": float}` |
| 7105 | SET | `SetVelocity` | `{"velocity": [vx,vy,omega], "duration": float}` |
| 7106 | SET | `SetArmTask` | `{"data": float}` |

**引用**: `unitree_sdk2py/g1/loco/g1_loco_api.py` (完整 API ID 定义)

#### FSM 状态机

```
FSM ID 0: ZeroTorque (零力矩)
FSM ID 1: Damp (阻尼/急停)
FSM ID 3: Sit (坐下)
FSM ID 4: Standby (待命)
FSM ID 500: Start (开始运动)
FSM ID 702: Lie2StandUp (躺→站)
FSM ID 706: Squat2StandUp / StandUp2Squat
```

**启动序列** (g1pilot 实现):
```python
robot.SetFsmId(4)      # → Standby
robot.Damp()           # → 安全阻尼
# ... 用户触发 balance ...
robot.SetStandHeight(height)  # 逐步升高
robot.BalanceStand(1)   # 平衡站立
robot.Start()           # FSM ID 500，开始运动
# 此后可以调用 Move(vx, vy, vyaw)
```

**引用**: `unitree_sdk2py/g1/loco/g1_loco_client.py:L60-L130` (所有高层方法)

#### SetVelocity 详解

```python
def SetVelocity(self, vx: float, vy: float, omega: float, duration: float = 1.0):
    """
    vx: 前进速度 (m/s)，正=前进
    vy: 侧移速度 (m/s)，正=左移 [G1 实际 vy≈0]
    omega: 旋转角速度 (rad/s)，正=逆时针
    duration: 指令持续时间 (s)，864000.0=持续移动
    """
```

**关键约束**:
- G1 不支持真正的侧移 (`vy` 会被内部控制器忽略或极小响应)
- 建议 `vy=0`，所有横向位移通过先旋转再前进实现
- `duration=864000.0` (10天) 表示持续移动，需外部发 `StopMove()` 才停

#### Move 方法

```python
def Move(self, vx, vy, vyaw, continous_move=False):
    duration = 864000.0 if continous_move else 1
    self.SetVelocity(vx, vy, vyaw, duration)
```

### 3.3 G1LocoClient ROS 2 节点

**源码**: `g1pilot/navigation/loco_client.py` (220 行)

#### 初始化流程

```python
ChannelFactoryInitialize(0, interface)  # CycloneDDS 通道
self.robot = LocoClient()
self.robot.SetTimeout(10.0)
self.robot.SetFsmId(4)   # 进入 Standby
self.robot.Init()
self.robot.Damp()        # 安全阻尼
```

#### 话题接口

| 话题 | 方向 | 类型 | 功能 |
|------|------|------|------|
| `/g1pilot/emergency_stop` | 订阅 | Bool | 紧急停止 → Damp |
| `/g1pilot/start` | 订阅 | Bool | 切换 FSM ID 4 |
| `/g1pilot/start_balancing` | 订阅 | Bool | 触发平衡站立序列 |
| `/g1pilot/joy` | 订阅 | Joy | **运动指令输入** |
| `/g1pilot/arms/enabled` | 发布 | Bool | 手臂控制开关 |
| `/base_height` | 订阅 | Float64 | 站立高度调整 |

#### Joy 消息速度映射

```python
# joystick_callback 中 (buttons[7]==1 且 balanced):
vx = round(msg.axes[1] * -0.5, 2)   # 前后轴 → 前进速度 [-0.5, 0.5]
vy = round(msg.axes[0] * -0.5, 2)   # 左右轴 → 侧移速度
yaw = round(msg.axes[2] * -0.5, 2)  # 旋转轴 → 偏航角速度
self.robot.Move(vx=vx, vy=vy, vyaw=yaw, continous_move=True)
```

**关键**: `buttons[8]=1` (nav2point 设置) 触发 `buttons[7]` 不同的控制路径。通过 `joy_mux` 切换手动/自动。

**引用**: `loco_client.py:L140-L180` (joystick_callback 核心), `loco_client.py:L190-L220` (entering_balancing)

### 3.4 nav2point 路径跟踪器

**源码**: `g1pilot/navigation/nav2point.py` (165 行)

这是 Track 3 最关键的"胶水"节点 —— 将 Path 消息转为 Joy 虚拟摇杆信号。

#### 控制参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `publish_rate` | 50 Hz | 控制循环频率 |
| `pos_kp` | 0.8 | 位置比例增益 |
| `yaw_kp` | 1.5 | 朝向比例增益 |
| `waypoint_tolerance` | 0.20 m | 中间路径点切换阈值 |
| `goal_tolerance` | 0.10 m | 终点到达阈值 |
| `vx_limit` | 0.6 m/s | 最大前进速度 |
| `vy_limit` | 0.6 m/s | 最大侧移速度 |
| `wz_limit` | 0.5 rad/s | 最大角速度 |
| `path_topic` | `/g1pilot/path` | 路径输入话题 |
| `joy_topic` | `/g1pilot/auto_joy` | Joy 输出话题 |
| `auto_enable_topic` | `/g1pilot/auto_enable` | 使能开关 |

#### 控制算法

```python
# 1. 位姿获取: 订阅 /lidar_odometry/pose_fixed (Odometry)
# 2. 路径消费: 订阅 /g1pilot/path (Path) → 提取 (x,y) 列表
# 3. Waypoint 跟踪:
dx = waypoint_x - robot_x
dy = waypoint_y - robot_y
dist = hypot(dx, dy)

# 如果到达当前 waypoint (< waypoint_tolerance)，切换下一个
# 如果到达终点 (< goal_tolerance)，停止

# 4. 速度计算 (世界坐标系):
vx_world = clamp(pos_kp * dx, -vx_limit, vx_limit)
vy_world = clamp(pos_kp * dy, -vy_limit, vy_limit)

# 5. 世界 → 机体坐标变换:
vx_body = cos(-yaw) * vx_world - sin(-yaw) * vy_world
vy_body = sin(-yaw) * vx_world + cos(-yaw) * vy_world

# 6. 朝向控制:
desired_yaw = atan2(dy, dx)
yaw_err = wrap_to_pi(desired_yaw - robot_yaw)
wz = clamp(yaw_kp * yaw_err, -wz_limit, wz_limit)

# 7. 输出 Joy:
axes[1] = clamp(-vx_body / vx_limit, -1, 1)  # 前后
axes[0] = clamp(-vy_body / vy_limit, -1, 1)  # 左右
axes[2] = clamp(-wz / wz_limit, -1, 1)       # 旋转
buttons[8] = 1  # 自动模式标志
```

**引用**: `nav2point.py:L95-L155` (loop 控制核心)

#### 位姿来源

```python
self.sub_odom = self.create_subscription(
    Odometry, '/lidar_odometry/pose_fixed', self.cb_odom, qos)
```

> **G1 适配**: deepglint 发布的是 `/localization_3d` (PoseStamped) 或 FAST-LIO 的 `/slam_odom` (Odometry)。需要通过 remap 或中间节点转换为 `/lidar_odometry/pose_fixed` 格式。

### 3.5 dijkstra_planner 2D 规划器

**源码**: `g1pilot/navigation/dijkstra_planner.py` (195 行)

g1pilot 内置的 2D Dijkstra 规划器，**在本项目中不使用**（被 jie_3d_nav 的 3D A* 完全替代）。

特性:
- 订阅 `/map` (OccupancyGrid) + `/lidar_odometry/pose_fixed` (Odometry)
- 输出 `/g1pilot/path` (Path)
- 支持: 占据膨胀、对角线移动、转弯代价、Catmull-Rom 平滑、路径捷径优化

**引用**: `dijkstra_planner.py:L45-L60` (参数声明)

### 3.6 RobotState 状态发布器

**源码**: `g1pilot/state/robot_state.py` (160 行)

通过 Unitree SDK2 的 `ChannelSubscriber("rt/lowstate", LowState_)` 直接订阅机器人底层状态，发布:

| 话题 | 类型 | 内容 |
|------|------|------|
| `/joint_states` | JointState | 29 DOF 关节位置 |
| `/g1pilot/imu` | Imu | 骨盆 IMU 姿态+角速度+加速度 |
| `/g1pilot/motor_state` | MotorStateList | 电机温度/电压/位置/速度 |
| TF `pelvis→imu_link` | TransformStamped | IMU 外参 |

**引用**: `robot_state.py:L40-L80` (G1JointIndex 定义 29 个关节)

### 3.7 接口对接方案

#### 3.7.1 话题重映射表

| jie_3d_nav 发布 | → | g1pilot 订阅 | 方式 |
|-----------------|---|-------------|------|
| `/planned_path` (Path) | → | `/g1pilot/path` | **launch remap** |
| `/start_navigation` (Bool) | → | `/g1pilot/auto_enable` | **launch remap** |
| `/stop_navigation` (Bool) | → | `/g1pilot/emergency_stop` | **launch remap + 逻辑适配** |

#### 3.7.2 位姿来源对接

| deepglint 发布 | nav2point 需要 | 解决方案 |
|---------------|---------------|---------|
| `/localization_3d` (PoseStamped) | `/lidar_odometry/pose_fixed` (Odometry) | 写一个轻量 relay 节点：PoseStamped → Odometry |
| 或 `/slam_odom` (Odometry) | 同上 | 直接 remap |

推荐方案: deepglint FAST-LIO 直接发布 Odometry 到 `/slam_odom`，通过 remap:
```yaml
remappings:
  - from: /lidar_odometry/pose_fixed
    to: /slam_odom
```

#### 3.7.3 启动序列

```bash
# 终端 1: deepglint 定位 (Track 1b)
ros2 launch open3d_loc localization_3d_g1.launch.py

# 终端 2: jie_3d_nav 地图+规划+Web (Track 2)
ros2 launch octo_planner web_test.launch.py \
    launch_controller:=false  # 不启动 d1_controller

# 终端 3: g1pilot 导航控制器 (Track 3)
ros2 launch g1pilot navigation_launcher.launch.py \
    path_topic:=/planned_path \
    auto_enable_topic:=/start_navigation
```

#### 3.7.4 Joy Mux 多路复用

g1pilot 的 `joy_mux.py` 支持在手动遥控和自动导航之间切换:

- `/g1pilot/joy` ← 手动摇杆
- `/g1pilot/auto_joy` ← nav2point 自动输出
- 当 `auto_enable=True` 时，auto_joy 优先

---


## 4. 集成架构与话题拓扑

### 4.1 完整数据流

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        ROS 2 容器 (运行时)                                │
│                                                                         │
│  [deepglint]                                                            │
│  livox_ros_driver2 → FAST-LIO2 → open3d_loc                           │
│       │                  │              │                               │
│       │            /slam_odom      /localization_3d                     │
│       │           (Odometry)       (PoseStamped)                        │
│       │                  │              │                               │
│       │                  ▼              ▼                               │
│       │          ┌──────────────────────────┐                           │
│       │          │  TF: map→odom→base_footprint                        │
│       │          └──────────────────────────┘                           │
│       │                                                                 │
│  [jie_3d_nav - Track 2]                                                │
│  map_package_manager → /octomap                                         │
│       │                    │                                            │
│       │              jie_path_node                                      │
│       │                    │                                            │
│       │              /planned_path ──────────┐                          │
│       │                    │                 │                          │
│       │    ┌───────────────┤                 │                          │
│       │    │               │                 │                          │
│       │  Web UI        /start_navigation     │                          │
│       │  :8080         /stop_navigation      │                          │
│       │                    │                 │                          │
│  [g1pilot - Track 3]      │                 │                          │
│       │                    ▼                 ▼                          │
│       │              nav2point ←── /g1pilot/path (remap)               │
│       │                    │    ←── /g1pilot/auto_enable (remap)       │
│       │                    │    ←── /slam_odom (remap)                 │
│       │                    │                                            │
│       │              /g1pilot/auto_joy                                  │
│       │                    │                                            │
│       │               joy_mux                                           │
│       │                    │                                            │
│       │              /g1pilot/joy                                       │
│       │                    │                                            │
│       │              loco_client                                        │
│       │                    │                                            │
│       │         LocoClient.Move(vx, 0, vyaw)                           │
│       │                    │                                            │
│       │              [Unitree G1 内置控制器]                              │
│       │                    │                                            │
│       │              ★ G1 实际行走 ★                                     │
│       │                                                                 │
└─────────────────────────────────────────────────────────────────────────┘
```

### 4.2 TF 树

```
map
 └── odom                    (open3d_loc / FAST-LIO)
      └── base_footprint     (机器人底盘)
           └── base_link
                ├── pelvis
                │    └── imu_link
                ├── mid360_link  (LiDAR)
                └── d455_link    (深度相机)
```

### 4.3 QoS 兼容性

| 话题 | jie_3d_nav QoS | g1pilot QoS | 兼容? |
|------|---------------|-------------|--------|
| `/planned_path` | transient_local + reliable | depth=10 (默认) | ✅ 兼容 |
| `/start_navigation` | reliable depth=10 | reliable depth=10 | ✅ |
| `/octomap` | transient_local + reliable | N/A (不订阅) | ✅ |
| `/slam_odom` | — | depth=10 | ✅ |

### 4.4 帧 ID 一致性检查清单

| 组件 | 发布帧 | 期望帧 | 状态 |
|------|--------|--------|------|
| open3d_loc TF | `map` → `odom` | `map` | ✅ |
| pcd_to_octomap_node | `map` | `map` | ✅ |
| jie_path_node path | `map` | — | ✅ |
| nav2point odom input | — | `map` (via TF) | 需确认 |

---

## 5. 关键技术风险与缓解策略

### 5.1 高风险

| # | 风险 | 影响 | 概率 | 缓解 |
|---|------|------|------|------|
| R1 | Open3D ARM64 编译失败 | Track 2 阻塞 | 高 | 预编译 wheel；或在 x86 笔记本跑 pcd_to_octomap_node 生成 map package 后拷贝到 G1 |
| R2 | OctoMap 内存溢出 (Orin NX 8GB) | Track 2 运行时崩溃 | 中 | 分辨率≥0.1m；PCD crop 到导航区域；使用 swap |
| R3 | nav2point vy 输出非零 | G1 行走不稳 | 中 | 在 nav2point 或 loco_client 中强制 vy=0 |

### 5.2 中风险

| # | 风险 | 影响 | 概率 | 缓解 |
|---|------|------|------|------|
| R4 | 位姿话题格式不匹配 | nav2point 无位姿 | 中 | 写 PoseStamped→Odometry relay |
| R5 | rosbridge_websocket 端口冲突 | Web UI 不可用 | 低 | 确认 9090/8080 端口无占用 |
| R6 | jie_path_node A* 超时 (大地图) | 规划失败 | 中 | 提高 max_iterations=500000；降低分辨率 |
| R7 | FSM 状态管理竞态 | G1 异常停止 | 低 | loco_client 已有 try/except + Damp 保护 |

### 5.3 低风险

| # | 风险 | 缓解 |
|---|------|------|
| R8 | Web UI 在移动端显示异常 | 用 PC 浏览器 |
| R9 | RViz 在 Jetson 无 GPU 加速 | 不在 G1 跑 RViz，用远程笔记本 |

---

## 6. 实施路线图

### Phase 1: 编译验证 (1-2 天)

```bash
# 1. 在 G1 上编译 jie_3d_nav
cd ~/g1_3d_ws/src
git clone https://github.com/leokkzzhzzz/jie_3d_nav_test.git
bash jie_3d_nav_test/install_deps_humble.sh

# 2. 编译 Open3D (ARM64) — 约 2-3 小时
git clone --depth 1 https://github.com/isl-org/Open3D.git /tmp/open3d
cd /tmp/open3d && mkdir build && cd build
cmake .. -DBUILD_CUDA_MODULE=OFF -DBUILD_GUI=OFF -DBUILD_EXAMPLES=OFF \
         -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_PYTHON_MODULE=OFF
make -j2 && sudo make install

# 3. 编译 jie_3d_nav
cd ~/g1_3d_ws
colcon build --packages-select jie_map_msgs jie_octomap octo_planner -j1
```

### Phase 2: PCD 导入验证 (0.5 天)

```bash
# 用 Track 1a 产出的 PCD 导入
ros2 launch jie_octomap import_pcd_map.launch.py
# GUI: 选择 ~/maps/scans.pcd → resolution=0.1 → 导入
# 验证: /octomap 发布, Web UI 可见
```

### Phase 3: 规划验证 (0.5 天)

```bash
ros2 launch octo_planner web_test.launch.py launch_controller:=false
# 浏览器: http://<G1_IP>:8080
# 测试: 点击起终点 → /planned_path 发布 → 路径几何合理
```

### Phase 4: g1pilot 集成 (1 天)

```bash
# 1. Clone g1pilot
cd ~/g1_3d_ws/src
git clone https://github.com/hucebot/g1pilot.git
pip install unitree_sdk2_python

# 2. 配置 remapping (创建 g1_nav.launch.py)
# /planned_path → /g1pilot/path
# /start_navigation → /g1pilot/auto_enable
# /slam_odom → /lidar_odometry/pose_fixed

# 3. 启动测试
ros2 launch g1pilot navigation_launcher.launch.py
```

### Phase 5: 端到端验证 (1 天)

```bash
# 同时启动三个组件
# 终端 1: deepglint 定位
# 终端 2: jie_3d_nav (地图+规划+Web)
# 终端 3: g1pilot (控制器)

# 验证清单:
# [ ] Web UI 设目标 → 规划 → 确认 → G1 开始行走
# [ ] Waypoint 逐一到达
# [ ] /stop_navigation 紧急停止正常
# [ ] FSM 状态链正确
```

---

## 7. 参考文献

### 源码引用

| 引用 ID | 文件 | 行号 | 内容 |
|---------|------|------|------|
| [S1] | `jie_octomap/src/pcd_to_octomap_node.cpp` | L40-165 | PCD→OctoMap 转换核心 |
| [S2] | `octo_planner/src/jie_path_node.cpp` | L86-140 | 规划参数声明 |
| [S3] | `octo_planner/src/jie_path_node.cpp` | L850-1050 | A* planAndPublish |
| [S4] | `octo_planner/src/d1_controller.cpp` | L210-460 | 路径跟踪控制器 |
| [S5] | `jie_octomap/web/main.js` | L1-50, 700-850 | Web UI 导航交互 |
| [S6] | `g1pilot/navigation/loco_client.py` | L40-220 | LocoClient ROS 封装 |
| [S7] | `g1pilot/navigation/nav2point.py` | L1-165 | 路径跟踪→Joy |
| [S8] | `unitree_sdk2py/g1/loco/g1_loco_client.py` | L1-130 | SDK LocoClient 实现 |
| [S9] | `unitree_sdk2py/g1/loco/g1_loco_api.py` | L1-35 | API ID 定义 |
| [S10] | `g1pilot/state/robot_state.py` | L40-160 | 29DOF 状态发布 |

### 外部引用

| 引用 | URL | 说明 |
|------|-----|------|
| [E1] | https://github.com/6-robot/jie_3d_nav | jie_3d_nav 上游仓库 |
| [E2] | https://github.com/hucebot/g1pilot | g1pilot 仓库 |
| [E3] | https://github.com/unitreerobotics/unitree_sdk2_python | Unitree SDK2 Python |
| [E4] | https://github.com/leokkzzhzzz/jie_3d_nav_test | Leo 的 jie_3d_nav fork |
| [E5] | https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID | deepglint 定位 |
| [E6] | http://docs.ros.org/lunar/api/octomap/html | OctoMap 官方文档 |
| [E7] | https://www.bilibili.com/video/BV1oz421v7tB | jie_3d_nav 介绍视频 |
| [E8] | https://markaicode.com/unitree-g1-first-python-walk-script/ | G1 SDK 入门教程 |
| [E9] | https://docs.openmind.org/robotics/unitree_g1_humanoid | G1 OM1 集成文档 |

### 项目文档引用

| 文件 | 说明 |
|------|------|
| `DEPLOYMENT_PLAN.md` | 四路部署计划（含 Track 2/3 操作步骤） |
| `CONTEXT.md` | 领域术语 + 5 条 ADR 架构决策 |
| `README.md` | Track 1a 实施记录 + 架构图 |
| `docs/PRD_TRACK1A.md` | Track 1a 详细 PRD |

---

*本报告由 Kiro AI 基于源码级分析自动生成。所有代码引用均来自实际仓库文件，可通过文件路径和行号定位验证。*
