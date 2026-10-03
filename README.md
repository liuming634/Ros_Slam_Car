# Ros_Slam_Car

基于 ROS 2 (Humble) 的 SLAM 小车项目：激光雷达 + IMU + 相机多传感器融合，实现同步定位与建图（SLAM）与自主导航。

## 功能特性

- **SLAM：Cartographer（激光雷达 2D 建图）**，融合 IMU，输出 `/map` 栅格地图与 TF（`map -> odom -> base_link`）
- **自主导航：Nav2**，map_server + AMCL 定位 + 规划/控制全套，仿真链路已接入（`launch/navigation.launch.py`）
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
│   ├── cartographer/         #    vi_car_2d.lua
│   └── nav2/                 #    vi_car_nav2_params.yaml
├── launch/                   # 启动入口，按"场景"分组
│   ├── simulation.launch.py  #    Gazebo 仿真建图全链路
│   ├── cartographer.launch.py#    Cartographer 建图（仿真/硬件通用）
│   ├── navigation.launch.py  #    Nav2 导航（AMCL + 规划控制）
│   └── include/              #    可复用片段（静态TF、硬件传感器链）
├── assets/                   # 运行产物（已 gitignore，只留目录占位）
│   ├── maps/                 #    地图文件（pgm / yaml / pbstream）
│   └── runtime/              #    其他运行时输出
├── docs/                     # 文档
│   ├── architecture.md       #    分层架构与数据流
│   ├── integration.md        #    SLAM/Nav2 对接手册：接口约定、对接点、踩坑登记
│   ├── slam.md               #    SLAM 方案与用法
│   ├── navigation.md         #    Nav2 数据流、参数与排障
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

## Nav2 导航仿真

前提：`assets/maps/` 里已有建图三件套（`vi_car.yaml / vi_car.pgm / vi_car.pbstream`，产出方法见上文"快速开始"）。

> 建图与导航是两条互斥链路，**不要同时启动**：
> 建图 = `simulation.launch.py` + `vi_car.urdf`，Cartographer 发 `map→odom`；
> 导航 = `navigation.launch.py` + `vi_car_nav.urdf`，Gazebo 里程计发 `odom→base_footprint`，AMCL 发 `map→odom`。

### ① 启动导航链

```bash
source install/setup.bash
ros2 launch launch/navigation.launch.py
```

一条命令拉起：Gazebo（同一房间场景）+ 导航版小车 + map_server + AMCL + Nav2 全套节点 + RViz（Nav2 官方视图）。
启动终端出现 `Managed nodes are active` 即就绪。

### ② 设初始位姿（每次启动后必做一次）

AMCL 收到初始位姿之前不发布 `map→odom` 的 TF，此时发目标必然失败。两种等价方式任选：

- RViz 工具栏 **2D Pose Estimate**：在小车真实位置（Gazebo 原点 = 地图中央附近）按车头方向拖一下；
- 命令行发同样的消息：

```bash
ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

### ③ 发导航目标（三种等价方式）

```bash
# A. RViz：工具栏 "Nav2 Goal" 在空地上点一下（最直观）

# B. 话题（= RViz 按钮发的那条消息；发完即退，不返回结果）
ros2 topic pub --once -w 1 /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -3.5, y: -1.5}, orientation: {w: 1.0}}}"

# C. action（阻塞到导航结束，退出码 0=到达，适合脚本判成败）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.5, y: -1.5}, orientation: {w: 1.0}}}}" \
  --feedback

# D. 官方 Python API（程序化多点巡航；Humble API 名与新版教程不同，见 docs/integration.md §4.4）
#    nav2_simple_commander 的 BasicNavigator：setInitialPose → waitUntilNav2Active → goToPose
```

- 目标点选位：地图可用范围约 x ∈ [-4.4, 4.9]、y ∈ [-2.5, 1.9]，**离墙和障碍箱至少 0.6 m**（膨胀半径 0.55）。参考点：`(-3.5, -1.5)` 左下、`(3.5, 1.5)` 右上、`(0, 1.5)` 顶部中间。
- 朝向换算：yaw 角 θ → 四元数 `z: sin(θ/2), w: cos(θ/2)`；朝 x 正方向就是 `w: 1.0`。

### ④ 排障速查

| 现象 | 原因 / 处理 |
|---|---|
| 发了目标车不动，启动终端刷 `Failed to send goal response (timeout)` | FastDDS 共享内存偶发故障：原样重发一次通常就好；反复出现就改走 UDP（见下） |
| RViz 有地图但报 TF 错 / 目标直接失败 | 多半忘了设初始位姿，回到 ② |
| 车在地图上的位置和 Gazebo 里对不上 | 定位漂移：再点一次 2D Pose Estimate 重新定位 |

UDP-only 配置（根治 response 超时）：存成 `~/fastdds_udp.xml`，并在**每个**跑 ROS 的终端里
`export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_udp.xml`（含启动导航链的终端）：

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_only</transport_id>
      <type>UDPv4</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_profile" is_default_profile="true">
    <rtps>
      <userTransports><transport_id>udp_only</transport_id></userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
```

数据流、参数逐项说明见 [docs/navigation.md](docs/navigation.md)。

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
仿真建图（Cartographer，已就绪）→ 仿真导航（AMCL 定位 + Nav2 规划控制，已接入）
→ 接入真实传感器 → 实车建图 → 实车导航
```

规划中的模块（serial / msgs / perception / control）见 [docs/roadmap.md](docs/roadmap.md)。

## 依赖

- Ubuntu 22.04 + ROS 2 Humble
- Cartographer（`ros2/cartographer_ros`，vendored 在 `src/algorithms/cartographer_ros/`）

> 本项目 SLAM 只用 Cartographer。原 vendored 的 VINS-Fusion 视觉方案（112MB 上游源码
> 与模型权重）已移除，`config/vins/` 与 `launch/vins.launch.py` 一并删除。
