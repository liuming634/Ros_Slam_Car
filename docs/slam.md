# slam SLAM 模块

本目录负责 ROS SLAM 小车的同时定位与建图（SLAM）功能。
本文档说明当前状态、Cartographer 在本项目中的定位与用法，以及**自动导航的方案与路线**。

---

## 1. 当前状态（2026-09）

- **主 SLAM：Cartographer（激光雷达）**，源码 vendored 在 `cartographer_ros/`
  （上游 `ros2/cartographer_ros`，git 还原后的干净上游版本，无嵌套 `.git`）。
- 数据链路（雷达为主，保留相机/IMU 接口）：

```
ydlidar_ros2_driver(/scan, laser_link)      ─┐
ybimu_driver -> madgwick(/imu/data, imu_link)├─> cartographer_node -> /map + /tf(map->odom->base_link)
camera_node(/camera/image_raw)   ← 保留接口，Cartographer 不消费
```

- 启动方式：顶层 `launch/simulation.launch.py`（一键 Gazebo 仿真建图）
  或 `launch/cartographer.launch.py`（需 Gazebo 已在别处运行）。
- **真实硬件驱动**：已接入 YDLidar X3（`src/drivers/lidar/ydlidar_ros2_driver`）
  与 YbImu（`src/drivers/imu/imu_ros2_device`），udev 固定 `/dev/ydlidar` / `/dev/myimu`。

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
| 相机 | `/camera/image_raw`（`sensor_msgs/Image`） | `camera_node` 发布，Cartographer 不消费，保留供后续视觉用 |

## 4. 使用说明

```bash
# 构建工作区（首次）。用 scripts/build.sh 会自动限流，避免 14G 内存机器被打爆
scripts/build.sh
source install/setup.bash

# 一键 Gazebo 仿真建图（房间场景 + 小车 + Cartographer）—— 推荐入口
ros2 launch launch/simulation.launch.py

# 只跑 Cartographer（Gazebo 需已在别处运行，传感器来源由 use_sim 切换）
ros2 launch launch/cartographer.launch.py

# 常用开关
ros2 launch launch/cartographer.launch.py use_camera:=false   # 无相机硬件时
ros2 launch launch/cartographer.launch.py use_imu:=false      # 不用 IMU
```

**换真实雷达**：改 `src/drivers/lidar/ydlidar_ros2_driver/params/ydlidar_x3.yaml`
（串口 `/dev/ydlidar`、115200 单通道）；同时按雷达量程调整
`config/cartographer/vi_car_2d.lua` 的 `min_range` / `max_range`。

### ⚠️ 排障：一直刷 `Queue waiting for data: (0, scan)` 且不产地图

这是本项目最容易踩的坑，**根因不在 Cartographer 配置，而在 Gazebo 场景**。

链路是这样断的：

```
空世界 → 雷达打不到任何东西 → /scan 的 ranges 全是 inf
      → ToPointCloudWithIntensities() 得到空点云
      → sensor_bridge.cpp HandleLaserScan() 里 if (points.points.empty()) return;
        ← 静默丢弃，没有任何 ERROR 日志
      → collator 的 scan 队列永远为空
      → OrderedMultiQueue::Dispatch() 每次都在空队列处提前返回
      → IMU 队列堆积超过 kMaxQueueSize(500)，才打出那条 WARNING
```

排查顺序：

```bash
# 1. 看 ranges 有没有有效回波（全是 inf 就是场景没障碍物）
ros2 topic echo /scan --once | grep -c '^- [0-9]'   # 0 = 场景是空的

# 2. 确认 Cartographer 真正开始消费了（出现这行 = 队列打通）
ros2 launch ... 2>&1 | grep "is available starting at"

# 3. 确认建图在推进
ros2 topic echo /submap_list --once    # submap_version 应持续增长，满 35 帧会新建 submap
```

**解法**：加载带障碍物的场景。`launch/simulation.launch.py` 默认已加载
`src/robot/vi_car_description/worlds/simulation.world`（内含 `simulation_test` 房间），
换场景用 `world:=<绝对路径.world>`。

> 注意：调试时若临时改用 `use_imu_data=false`，那条 WARNING 会消失，但**地图依然出不来**
> —— 因为 WARNING 只在「有队列堆积超过 500 条」时才打印，IMU 关掉后没人堆积了。
> 别被"没告警"骗了，要以 `submap_list` 的 `submap_version` 是否增长为准。

### 保存地图（建图完成后）

启动时加 `save_map:=true`，进程退出时会把 pbstream 写到 `assets/maps/vi_car.pbstream`：

```bash
ros2 launch launch/simulation.launch.py save_map:=true
# ...遥控走一圈，然后 Ctrl+C 结束
```

> 注意：pbstream 是在 `cartographer_node` 收到 SIGINT 优雅退出时才落盘的。
> 用 `kill -9` 强杀不会写出文件。

转成栅格地图（给 AMCL / Nav2 用）：

```bash
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
  -pbstream_filename assets/maps/vi_car.pbstream
# 产物固定落在【当前工作目录】，手动搬到目标位置：
mv map.pgm map.yaml assets/maps/
sed -i 's/^image: map.pgm/image: vi_car.pgm/' assets/maps/map.yaml
```

> ⚠️ 本机编译的这个工具**只有 `-pbstream_filename` 和 `-resolution` 两个 flag**，
> 输出文件名硬编码为 CWD 下的 `map.pgm` / `map.yaml`（没有 `--ros_map_filename`，
> 传了会被忽略）。`map.yaml` 里的 `origin` 是真实地图原点偏移（如 `[-0.3, -5.65, 0]`），
> **别手写覆盖**，直接用工具生成的。

---

## 5. 自动导航方案（后续路线）

```
传感器：2D 雷达 + 轮式编码器 + IMU
  ├─ 建图：Cartographer → 2D 栅格地图 /map（当前已就绪）
  ├─ 定位：AMCL（在已保存地图里定位）
  ├─ 规划：Nav2 / move_base（全局 + 局部路径规划）
  ├─ 控制：底盘驱动节点（差速/阿克曼）
  └─（可选）视觉里程计作为一路输入 robot_localization 融合
```

| 阶段 | 内容 | 依赖 |
|---|---|---|
| ① 当前 | Cartographer 建图链路（sim 联调） | 雷达 / IMU 硬件 |
| ② 接入真传感器 | lidar/imu 串口驱动 | 购买/手头设备 |
| ③ 建图 | Cartographer 实车建图 + 保存地图 | ② |
| ④ 定位 | AMCL 地图内定位 | ③ 保存的地图 |
| ⑤ 导航 | Nav2 路径规划 + 底盘控制 | ③④ + 底盘驱动 |
| ⑥ 融合（可选） | IMU / 轮式里程计多源融合（robot_localization） | 全部跑通后 |

---

## 6. 目录结构

```
src/algorithms/
└── cartographer_ros/          # vendored 源码（无嵌套 .git），上游 ros2/cartographer_ros，含
    ├── cartographer_ros/       #   主包：cartographer_node 等
    ├── cartographer_ros_msgs/  #   消息包
    └── cartographer_rviz/      #   RViz 插件
```

小车自身的 SLAM 配置在顶层 `config/`：
- `cartographer/vi_car_2d.lua` —— Cartographer 2D 建图配置（激光 + 可选 IMU）
