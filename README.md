# Ros_Slam_Car

基于 ROS 2 (Humble) 的 SLAM 小车项目：激光雷达 + IMU + 相机多传感器融合，实现同步定位与建图（SLAM）与自主导航。

## 功能特性

- **SLAM：Cartographer（激光雷达 2D 建图）**，融合 IMU，输出 `/map` 栅格地图与 TF（`map -> odom -> base_link`）
- **仿真优先**：Gazebo 传感器插件直供 `/scan` + `/imu/data`，无需硬件即可跑通建图全链路
- **硬件可切**：`use_sim:=false` 一键切换到 YDLidar 雷达 + YbImu + 相机真实驱动

## 目录结构

顶层只有 6 个目录，按职责分离：

```
Ros_Slam_Car/
├── src/                      # 全部代码，colcon 唯一扫描根；按"层"分组
│   ├── drivers/              # ① 硬件驱动层：传感器 → ROS 话题
│   │   ├── lidar/            #    ydlidar_sdk + ydlidar_ros2_driver
│   │   ├── imu/              #    imu_ros2_device
│   │   └── camera/           #    USB 摄像头 / FPV 图传
│   ├── algorithms/           # ② 算法层：话题 → map / odom / tf
│   │   └── cartographer_ros/ #    Cartographer（3 个上游包）
│   └── robot/                # ③ 本体描述层：坐标系唯一真源
│       └── vi_car_description/   # urdf/ worlds/ models/ rviz/
├── config/                   # 项目级配置，按"域"分组
│   └── cartographer/         #    vi_car_2d.lua
├── launch/                   # 启动入口，按"场景"分组
│   ├── simulation.launch.py  #    Gazebo 仿真全链路
│   ├── cartographer.launch.py#    Cartographer 建图（仿真/硬件通用）
│   └── include/              #    可复用片段（静态TF、硬件传感器链）
├── assets/                   # 运行产物（已 gitignore，只留目录占位）
│   ├── maps/                 #    地图文件（pgm / yaml / pbstream）
│   └── runtime/              #    其他运行时输出
├── docs/                     # 文档
│   ├── architecture.md       #    分层架构与数据流
│   ├── slam.md               #    SLAM 方案与用法
│   ├── hardware.md           #    硬件接线与驱动
│   ├── config.md             #    配置项详解
│   └── roadmap.md            #    规划中模块
└── scripts/                  # 构建与运维脚本
```

架构约定与数据流详见 [docs/architecture.md](docs/architecture.md)。

## 快速开始

```bash
cd /home/lm/Ubuntu/code/project/Ros_Slam_Car

# 构建工作区（首次）。scripts/build.sh 已内置并发限流：
# 本机 32 核但只有 14G 内存，裸跑 colcon build 会并行展开到满核，瞬间打爆内存卡死。
scripts/build.sh
source install/setup.bash

# 一键 Gazebo 仿真建图（Gazebo 房间场景 + 小车 + Cartographer）—— 推荐入口
ros2 launch launch/simulation.launch.py

# 只跑 Cartographer（需 Gazebo 已在别处运行；传感器来源由 use_sim 切换）
ros2 launch launch/cartographer.launch.py
```

车静止时 Cartographer 建不出地图，另开终端遥控：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.4
```

> 排障：一直刷 `Queue waiting for data: (0, scan)` 表示雷达没打到障碍物
> （多为 Gazebo 用了空世界），详见 [docs/slam.md](docs/slam.md) 第 4 节。

## 常用开关

```bash
# 仿真 / 真实硬件切换
ros2 launch launch/cartographer.launch.py use_sim:=false

# 换 Gazebo 场景（必须是有障碍物的场景，否则雷达全是 inf、建不出图）
ros2 launch launch/simulation.launch.py world:=/abs/path/to/your.world

# 退出时把建好的地图存成 pbstream 到 assets/maps/
ros2 launch launch/simulation.launch.py save_map:=true

# 关闭不需要的传感器（use_sim:=false 时生效）
ros2 launch launch/cartographer.launch.py use_camera:=false
ros2 launch launch/cartographer.launch.py use_imu:=false

# 关闭可视化
ros2 launch launch/cartographer.launch.py use_rviz:=false
ros2 launch launch/simulation.launch.py gui:=false   # 无头仿真
```

## 常用脚本

```bash
scripts/build.sh              # 限流构建（默认总并发 2）
scripts/build.sh --clean      # 清干净再全量构建
scripts/build.sh -j4          # 机器空闲时可放宽到 4
scripts/build.sh camera       # 只构建指定包
scripts/clean.sh              # 清 build/install/log + 运行时产物
scripts/clean.sh --all        # 额外清空 assets/maps（地图会丢）
```

## 自动导航路线

```
建图（Cartographer，当前已就绪）→ 接入真实传感器 → 实车建图
→ AMCL 地图定位 → Nav2 路径规划 + 底盘控制
```

规划中的模块（serial / msgs / perception / control / navigation）见 [docs/roadmap.md](docs/roadmap.md)。

## 依赖

- Ubuntu 22.04 + ROS 2 Humble
- Cartographer（`ros2/cartographer_ros`，vendored 在 `src/algorithms/cartographer_ros/`）

> 本项目 SLAM 只用 Cartographer。原 vendored 的 VINS-Fusion 视觉方案（112MB 上游源码
> 与模型权重）已移除，`config/vins/` 与 `launch/vins.launch.py` 一并删除。