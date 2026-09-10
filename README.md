# Ros_Slam_Car

基于 ROS 2 (Humble) 的 SLAM 小车项目：激光雷达 + IMU + 相机多传感器融合，实现同步定位与建图（SLAM）与自主导航。

## 功能特性

- **主 SLAM：Cartographer（激光雷达 2D 建图）**，融合 IMU，输出 `/map` 栅格地图与 TF（`map -> odom -> base_link`）
- **备选 SLAM：VINS-Fusion（纯视觉单目 VIO）**，验证相机数据链路
- **传感器数据链路**：`lidar_node`(/scan) + `imu_node`(/imu/data) + `camera_node`(/camera/image_raw)
- **模块化工作区**：`src/` 下按功能划分 hardware / perception / slam / navigation / control / msgs / common / ui

## 目录结构

```
Ros_Slam_Car/
├── src/
│   ├── hardware/     # 硬件驱动：camera（USB/FPV 图传）、imu、lidar、serial
│   ├── perception/   # 感知模块（目标检测、跟踪、点云处理）
│   ├── slam/         # SLAM 模块：cartographer_ros（vendored）、VINS-Fusion-ROS2-humble
│   ├── navigation/   # 导航模块（全局/局部路径规划、避障）
│   ├── control/      # 控制模块（底盘控制、PID、路径跟踪）
│   ├── msgs/         # 自定义消息 / 服务 / 动作
│   ├── common/       # 通用工具库
│   └── ui/           # 用户界面（RViz 等）
├── config/vi_car/    # 项目级 SLAM 配置（cartographer + VINS）
├── launch/           # 一键启动文件
├── urdf/             # 小车模型与静态 TF
├── maps/             # 地图文件
├── data/             # 运行时数据（VINS 轨迹等）
├── docs/             # 文档
├── scripts/          # 脚本
└── test/             # 测试
```

## 快速开始

```bash
# 构建工作区（首次）
cd /home/lm/Ubuntu/code/Ros_Slam_Car
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 一键启动 Cartographer SLAM（sim 雷达 + sim IMU + 相机，无需硬件可跑通链路）
ros2 launch launch/cartographer.launch.py

# 备选：相机 + VINS-Fusion（视觉 SLAM）
ros2 launch launch/vi_car.launch.py
```

## 常用开关

```bash
ros2 launch launch/cartographer.launch.py use_camera:=false   # 无相机硬件时
ros2 launch launch/cartographer.launch.py use_imu:=false      # 不用 IMU
ros2 launch launch/cartographer.launch.py use_rviz:=false     # 关闭 RViz
```

## 自动导航路线

```
建图（Cartographer，当前已就绪）→ 接入真实传感器 → 实车建图
→ AMCL 地图定位 → Nav2 路径规划 + 底盘控制
```

## 依赖

- Ubuntu 22.04 + ROS 2 Humble
- Cartographer（`ros2/cartographer_ros`，vendored 在 `src/slam/cartographer_ros/`）
- VINS-Fusion（`zinuok/VINS-Fusion-ROS2`，vendored 在 `src/slam/VINS-Fusion-ROS2-humble/`）