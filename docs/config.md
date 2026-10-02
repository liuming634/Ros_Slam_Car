# vi_car - 小车 SLAM 配置

顶层 `config/` 存放 ROS SLAM 小车的**项目级** SLAM 配置（包私有参数不在这里，见
[architecture.md](architecture.md) 的分工说明）：

- `cartographer/` —— **主 SLAM：Cartographer 2D 建图配置**（激光雷达为主，可选 IMU）

---

## Cartographer 配置详解

配置文件：`cartographer/vi_car_2d.lua`

### 数据链路

```
lidar_node(/scan) ─┐
imu_node(/imu/data)├─> cartographer_node(读取 vi_car_2d.lua) -> /map + /tf
camera_node(/camera/image_raw)   ← 保留接口，Cartographer 不消费
```

### 坐标系

`map`(全局) -> `odom`(cartographer 发布) -> `base_link` -> `laser_link` / `camera_link` / `imu_link`
（静态 TF 来自 `src/robot/vi_car_description/urdf/vi_car.urdf`，经 `robot_state_publisher` 发布）

### 启动方式

```bash
cd /home/lm/Ubuntu/code/project/Ros_Slam_Car
scripts/build.sh          # 已内置并发限流，别直接裸跑 colcon build
source install/setup.bash

# 一键 Gazebo 仿真建图（自带房间场景）
ros2 launch launch/simulation.launch.py

# 只跑 Cartographer（需 Gazebo 已在别处运行）
ros2 launch launch/cartographer.launch.py
```

### 配置要点（vi_car_2d.lua）

| 项 | 值 | 说明 |
|----|----|------|
| `map_frame` / `odom_frame` | map / odom | TF 坐标系 |
| `tracking_frame` / `published_frame` | imu_link / base_link | tracking 是被估计的 IMU 帧（IMU 数据不做坐标系变换，须直接落在该帧）；cartographer 发布 odom->base_link |
| `provide_odom_frame` | true | 纯雷达 SLAM：由 cartographer 提供 odom |
| `num_laser_scans` | 1 | 订阅 `/scan` |
| `use_imu_data` | true | 使用 IMU（launch 里 `/imu` remap 到 `/imu/data`）；无 IMU 改 false |
| `min_range` / `max_range` | 0.15 / 12.0 | 按雷达实际量程调整（RPLIDAR A1 等） |
| `use_odometry` | false | 接入轮式里程计后改 true |

