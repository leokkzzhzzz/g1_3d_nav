# G1 3D Navigation — Deployment Plan

## Architecture

```
ROS 1 容器 (离线, 一次性)               ROS 2 容器 (运行时)
┌─────────────────────────┐            ┌──────────────────────────────┐
│ HongTu fastlio2         │            │ deepglint livox_ros_driver2  │
│ + GTSAM 回环            │   map.pcd  │   ↓                          │
│ MID360 → mapping → PCD  │ ─────────→ │ FAST-LIO2 (odometry)         │
└─────────────────────────┘            │   ↓                          │
                                       │ open3d_loc (重定位)           │
                                       │   ↓                          │
                                       │ octomap_server → /octomap    │
                                       │   ↓                          │
                                       │ jie_3d_nav (规划 + Web UI)   │
                                       │   ↓  /planned_path           │
                                       │ g1pilot (控制 + SDK)         │
                                       │   ↓  LocoClient              │
                                       │ Unitree G1                   │
                                       └──────────────────────────────┘
```

两容器之间**唯一接口是 PCD 文件**，通过宿主机目录挂载共享。

---

## Track 1a: HongTu ROS 1 离线建图

**Goal**: 在 ROS 1 独立容器中运行 HongTu FAST-LIO2（含 GTSAM 回环），产出高质量 map.pcd。

### 1a.1 准备 ROS 1 容器

```bash
# 拉取 ROS 1 Noetic 镜像或使用现有
docker pull ros:noetic-ros-base

# 启动容器，挂载宿主机目录用于保存 PCD
docker run -it --name g1_mapper \
  --network host \
  -v /home/nvidia/maps:/maps \
  ros:noetic-ros-base
```

### 1a.2 Clone 并编译 HongTu

```bash
# 容器内
cd ~/catkin_ws/src
git clone https://github.com/yuanqizhiti/HongTu.git
# 只取 fastlio2 部分:
# G1Nav2D/src/fastlio2/

# 安装依赖
sudo apt install ros-noetic-gtsam ros-noetic-pcl-ros
# Livox SDK2 + livox_ros_driver2 (ROS 1 版本)

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 1a.3 配置 MID360

确认 LiDAR IP、外参配置与实机一致。HongTu 默认支持 MID360。

### 1a.4 执行建图

```bash
# 启动建图
roslaunch fastlio mapping.launch

# 遥控 G1 在目标环境中走一圈（覆盖所有楼层、走廊、房间）

# 保存 PCD 地图
rosservice call /save_map "save_path: '/maps/target_env.pcd'"
```

### 1a.5 验证

- [ ] HongTu FAST-LIO2 启动无报错
- [ ] `/cloud_registered` 点云匹配场景几何
- [ ] GTSAM 回环检测触发（走闭环路径时点云自动对齐）
- [ ] `/save_map` 服务返回成功
- [ ] `/maps/target_env.pcd` 文件存在且大小合理（> 几 MB）
- [ ] PCD 在 CloudCompare 或 pcl_viewer 中可视化无异常

### 1a.6 收尾

建图完成后关闭容器。PCD 文件保留在宿主机 `/home/nvidia/maps/` 下，供 ROS 2 容器读取。

---

## Track 1b: deepglint ROS 2 运行时定位

**Goal**: 在 ROS 2 容器中运行 deepglint 的 livox 驱动 + FAST-LIO2 odometry + open3d_loc 重定位，加载 Track 1a 的 PCD 实现全局定位。

### 1b.1 准备 ROS 2 容器

```bash
# 使用与仿真一致的镜像
docker run -it --name g1_runtime \
  --network host \
  -v /home/nvidia/maps:/maps \
  fishros2/ros:humble-desktop-full
```

### 1b.2 Clone 并编译 deepglint

```bash
cd ~/g1_3d_ws/src
git clone https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID.git
cd FAST_LIO_LOCALIZATION_HUMANOID
git checkout humble  # ROS 2 Humble 分支

# 安装依赖
# - Livox-SDK2
# - Open3D (ARM 版或自行编译)
# - PCL, Eigen

cd ~/g1_3d_ws
colcon build --symlink-install
source install/setup.bash
```

### 1b.3 配置 MID360 (倒装)

`livox_ros_driver2/config/MID360_config.json`:
```json
{
  "lidar_configs": [{
    "ip": "<G1_MID360_IP>",
    "extrinsic_parameter": {
      "roll": 180.0,
      "pitch": 0.0,
      "yaw": 0.0,
      "x": 0.0, "y": 0.0, "z": 0.0
    }
  }]
}
```

### 1b.4 启动定位

```bash
# 将 Track 1a 的 PCD 放入 open3d_loc 的地图目录
cp /maps/target_env.pcd ~/g1_3d_ws/src/FAST_LIO_LOCALIZATION_HUMANOID/data/

# 启动定位
ros2 launch open3d_loc localization_3d_g1.launch.py
```

### 1b.5 验证

- [ ] `/livox/lidar` CustomMsg 正常发布（`ros2 topic hz` > 0）
- [ ] `/livox/imu` Imu 重力方向正确（静止时 z 分量 ≈ +9.81）
- [ ] FAST-LIO2 odometry 不漂移
- [ ] `/localization_3d` 发布（PoseStamped）
- [ ] `/localization_3d_confidence` > 0.8（ICP fitness）
- [ ] TF 树完整：`map → odom → base_footprint`
- [ ] 在 RViz 中验证机器人位姿与实际位置一致

---

## Track 2: jie_3d_nav 地图导入 + 规划 + Web

**Goal**: PCD → OctoMap 转换，map package 管理，3D A* 规划，Web 可视化。与 Track 1b 共享同一 ROS 2 容器。

### 2.1 Clone 并编译

```bash
cd ~/g1_3d_ws/src
git clone https://github.com/leokkzzhzzz/jie_3d_nav_test.git

cd jie_3d_nav_test
bash install_deps_humble.sh

cd ~/g1_3d_ws
colcon build --packages-select jie_map_msgs jie_octomap octo_planner
source install/setup.bash
```

### 2.2 PCD → OctoMap 导入

```bash
ros2 launch jie_octomap import_pcd_map.launch.py
# GUI: 选择 /maps/target_env.pcd → 配置分辨率 → 导入
```

- [ ] `/octomap` 话题发布
- [ ] `/octomap_occupied_markers` 体素可视化正常
- [ ] OctoMap 在 RViz 中与原始 PCD 场景一致

### 2.3 保存 Map Package

```bash
ros2 launch jie_octomap map_manager.launch.py
# map_viewer_gui → File → Save Package → ~/maps/B1/
```

- [ ] `meta.yaml` + `octomap_msg.npz` + `layers.npz` 写入
- [ ] 重新加载 package 后 OctoMap 内容一致

### 2.4 Web 3D 可视化

```bash
ros2 launch jie_octomap web_octomap.launch.py map_package:=~/maps/B1
```

- [ ] `http://<G1_IP>:8080` 可访问
- [ ] 各图层 toggle 正常
- [ ] 点击可通行栅格设置起终点

### 2.5 3D 路径规划测试

配置 `octo_planner/config/nav_params.yaml`:
```yaml
map_package_dir: /home/nvidia/maps/B1
relocalization_bin_file: ""
relocalization_pcd_file: ""
show_rviz: true
```

```bash
ros2 launch octo_planner web_test.launch.py
```

- [ ] Web UI 设起终点 → `/planned_path` 发布
- [ ] 路径不穿墙，几何合理
- [ ] 确认导航弹窗出现
- [ ] `/start_navigation` 发布 Bool=true

---

## Track 3: g1pilot 控制器接入

**Goal**: `/planned_path` → waypoint 跟踪 → Unitree LocoClient → G1 行走。

### 3.1 获取 g1pilot

```bash
cd ~/g1_3d_ws/src
git clone https://github.com/hucebot/g1pilot.git
```

### 3.2 接口对接

| jie_3d_nav | → | g1pilot | 方式 |
|---|---|---|---|
| `/planned_path` | → | `/g1pilot/path` | topic remap |
| `/start_navigation` (Bool) | → | `/g1pilot/auto_enable` | topic remap |
| `/stop_navigation` (Bool) | → | auto_enable=false | 同上 |

### 3.3 位姿来源确认

nav2point 订阅 `/lidar_odometry/pose_fixed` — 需确认 deepglint 的 `/localization_3d` 或 FAST-LIO `slam_odom` 能适配。

### 3.4 安装 Unitree SDK

```bash
# Python SDK (匹配 g1pilot 风格)
pip install unitree_sdk2_python
# 或 clone:
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
```

### 3.5 端到端测试

```bash
# 终端 1: deepglint 定位
ros2 launch open3d_loc localization_3d_g1.launch.py

# 终端 2: jie_3d_nav
ros2 launch octo_planner web_test.launch.py

# 终端 3: g1pilot 控制器
ros2 launch g1pilot navigation_launcher.launch.py
```

- [ ] Web UI 设目标 → 规划 → 确认 → G1 开始行走
- [ ] Waypoint 逐一到达，最终停在目标点
- [ ] `/stop_navigation` 紧急停止正常
- [ ] FSM 状态管理正确（Damp→StandUp→Start→行走）

---

## Integration Checkpoint (All Tracks)

### TF 树验证

```
map → odom → base_footprint → base_link → mid360_link
  ↑        ↑
open3d   FAST-LIO
_loc     odometry
```

### 话题流验证

```bash
ros2 topic info -v /octomap          # octomap_server
ros2 topic info -v /planned_path     # jie_path_node
ros2 topic info -v /localization_3d  # open3d_loc
ros2 topic info -v /livox/lidar      # deepglint driver
```

### Frame ID 一致性

- [ ] OctoMap `frame_id` == open3d_loc `map` frame == FAST-LIO `map` frame
- [ ] Web UI TF 解析 `map → base_footprint` 正常
- [ ] nav2point 位姿帧与 deepglint 输出帧一致

---

## Risk Register

| Risk | Impact | Mitigation |
|------|--------|-----------|
| Open3D ARM 编译失败 (Orin NX) | Track 1b blocked | 使用 x86 笔记本跑定位，Orin NX 只跑控制；或预编译 open3d wheel |
| deepglint Humble 分支不完整 | Track 1b blocked | 切 ROS 1 分支 + ros1_bridge，或改用原版 FAST_LIO_LOCALIZATION |
| HongTu GTSAM 依赖缺失 | Track 1a blocked | 独立安装 gtsam，或使用 HongTu 提供的 Dockerfile |
| G1 LocoClient Python 绑定不可用 | Track 3 blocked | 改用 C++ SDK，或直接用 CycloneDDS 发包 |
| OctoMap 内存超 Orin NX 限制 | Track 2 slow | 分辨率降到 0.1m，crop 到导航相关区域 |
| MID360 IP 配置未知 | Track 1a/1b startup | 先 SSH 进 G1 查看网络配置，找到 MID360 静态 IP |
