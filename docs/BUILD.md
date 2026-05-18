# 编译指南

## 1. 安装依赖
```bash
bash install_deps.sh
```

## 2. 编译 Open3D v1.4.1 (ARM64)
容器内编译或使用预编译包。安装到 `/usr/local/lib` 和 `/usr/local/include`。

## 3. 编译 LAPACKE
```bash
cd /tmp && wget https://github.com/Reference-LAPACK/lapack/archive/v3.9.0.tar.gz
tar xzf v3.9.0.tar.gz && cd lapack-3.9.0 && cp make.inc.example make.inc
make lapackelib -j1
cp liblapacke.a liblapacke.so* /usr/lib/aarch64-linux-gnu/
```

## 4. catkin_make
```bash
source /opt/ros/noetic/setup.bash
cd ~/g1_nav_ws
catkin_make -j1
```

## 5. 验证
```bash
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch  # 驱动
roslaunch fast_lio mapping_mid360_g1.launch      # 建图
roslaunch open3d_loc open3d_loc_g1.launch         # 定位
```
