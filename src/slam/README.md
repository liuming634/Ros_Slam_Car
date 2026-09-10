# slam SLAM 模块

本目录负责 ROS SLAM 小车的同时定位与建图（SLAM）功能。
本文档说明当前状态、Cartographer 在本项目中的定位与用法，以及**自动导航的方案与路线**。

---

## 1. 当前状态（2026-09）

- **主 SLAM：Cartographer（激光雷达）**，源码 vendored 在 `cartographer_ros/`
  （上游 `ros2/cartographer_ros`，git 还原后的干净上游版本，无嵌套 `.git`）。
- 数据链路（雷达为主，保留相机/IMU 接口）：

```
lidar_node(/scan) ─┐
imu_node(/imu/data)├─> cartographer_node -> /map + /tf(map->odom->base_link)
camera_node(/camera/image_raw)   ← 保留接口，Cartographer 不消费
```

- 启动方式：顶层 `launch/cartographer.launch.py`（一键启动 SLAM + 雷达 + IMU + 相机 + TF）。
- **备选 SLAM：VINS-Fusion**（视觉惯性），源码在 `VINS-Fusion-ROS2-humble/`，
  用顶层 `launch/vi_car.launch.py` 单独启动（相机 + VINS）。
- **⚠️ 注意**：`lidar_node` / `imu_node` 目前是 **sim 模拟模式**（无硬件也能跑通链路）；
  接入真实雷达 / IMU 后在 `src/hardware/lidar`、`src/hardware/imu` 里实现串口驱动。

---

## 2. Cartographer 在本项目的定位

**Cartographer 是激光 SLAM**：融合激光（+ IMU/里程计）实时建 2D 栅格地图并输出位姿，
是地面小车导航的标准方案。

| 它能做 | 配套需要 |
|---|---|
| 实时位姿估计（`odom` 帧） | TF：`map -> odom -> base_link -> {laser, camera, imu}` |
| 建可导航 2D 栅格地图 `/map` | `cartographer_occupancy_grid_node` |
| 回环检测修正漂移 | 保存地图（pbstream）后可转栅格地图用于 AMCL 定位 |
| 融合 IMU / 轮式里程计 | 接入后置 `use_imu_data` / `use_odometry` |

## 3. 坐标系与数据接口

```
map(全局) -> odom(cartographer 发布) -> base_link -> laser_link
                                              ├─ camera_link
                                              └─ imu_link
```

| 接口 | 话题 | 说明 |
|---|---|---|
| 雷达 | `/scan`（`sensor_msgs/LaserScan`） | `lidar_node` 发布，Cartographer 主输入 |
| IMU | `/imu/data`（`sensor_msgs/Imu`） | `imu_node` 发布，launch 里 remap 给 Cartographer 的 `/imu` |
| 相机 | `/camera/image_raw`（`sensor_msgs/Image`） | `camera_node` 发布，保留供 VINS / 后续视觉用 |

## 4. 使用说明

```bash
# 构建工作区（首次，含 vendored cartographer_ros 3 个包）
cd /home/lm/Ubuntu/code/Ros_Slam_Car
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 一键启动 Cartographer（默认 sim 雷达 + sim IMU + 相机，可加 use_rviz:=false 关可视化）
ros2 launch launch/cartographer.launch.py

# 备选：相机 + VINS（视觉 SLAM）
ros2 launch launch/vi_car.launch.py

# 常用开关
ros2 launch launch/cartographer.launch.py use_camera:=false   # 无相机硬件时
ros2 launch launch/cartographer.launch.py use_imu:=false      # 不用 IMU
```

**换真实雷达**：改 `src/hardware/lidar/config/lidar_params.yaml` 的 `lidar_source: serial`
并实现 `lidar_node.cpp` 串口分支；同时按雷达量程调整
`config/vi_car/cartographer/vi_car_2d.lua` 的 `min_range` / `max_range`。

**保存地图（建图完成后）**：给 `cartographer_node` 加
`-save_state_filename /path/to/map.pbstream`（launch 里加参数），之后用
`cartographer_pbstream_to_ros_map` 转成栅格地图给 AMCL / Nav2。

---

## 5. VINS-Fusion（备选 SLAM）

**VINS-Fusion 是视觉惯性里程计（VIO）**：实时估算设备相对起点的 6 自由度位姿，稀疏建图。
它**不能**建可导航栅格地图，对地面小车导航只能当里程计配角。
当前保留的纯视觉单目配置（`config/vi_car/vi_car.yaml`）用于验证相机数据链路；
单目 + 无 IMU 存在尺度不确定，接入 IMU 或双目/深度后可作视觉里程计输入。

| 它能做 | 它不能做 |
|---|---|
| 实时位姿/里程计估计（GPS 失效环境） | 建可导航地图（栅格/稠密） |
| 稀疏特征点图 + 回环修正漂移 | 在地图里定位（AMCL 那种） |
| 作为多传感器融合中的一路里程计 | 路径规划、避障、控制 |

---

## 6. 自动导航方案（后续路线）

```
传感器：2D 雷达 + 轮式编码器 + IMU
  ├─ 建图：Cartographer → 2D 栅格地图 /map（当前已就绪）
  ├─ 定位：AMCL（在已保存地图里定位）
  ├─ 规划：Nav2 / move_base（全局 + 局部路径规划）
  ├─ 控制：底盘驱动节点（差速/阿克曼）
  └─（可选）VINS-Fusion 作为视觉里程计输入 robot_localization 融合
```

| 阶段 | 内容 | 依赖 |
|---|---|---|
| ① 当前 | Cartographer 建图链路（sim 联调） | 雷达 / IMU 硬件 |
| ② 接入真传感器 | lidar/imu 串口驱动 | 购买/手头设备 |
| ③ 建图 | Cartographer 实车建图 + 保存地图 | ② |
| ④ 定位 | AMCL 地图内定位 | ③ 保存的地图 |
| ⑤ 导航 | Nav2 路径规划 + 底盘控制 | ③④ + 底盘驱动 |
| ⑥ 融合（可选） | VINS/IMU/轮式里程计多源融合（robot_localization） | 全部跑通后 |

---

## 7. 目录结构

```
slam/
├── cartographer_ros/          # vendored 源码（无嵌套 .git），上游 ros2/cartographer_ros，含
│   ├── cartographer_ros/       #   主包：cartographer_node 等
│   ├── cartographer_ros_msgs/  #   消息包
│   └── cartographer_rviz/      #   RViz 插件
├── VINS-Fusion-ROS2-humble/   # vendored 源码，上游 zinuok/VINS-Fusion-ROS2（备选）
└── README.md                  # 本文档
```

小车自身的 SLAM 配置在顶层 `config/vi_car/`：
- `cartographer/vi_car_2d.lua` —— Cartographer 2D 建图配置（激光 + 可选 IMU）
- `vi_car.yaml` / `cam0_pinhole.yaml` —— VINS 纯视觉单目配置
