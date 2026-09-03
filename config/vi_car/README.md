# vi_car - 小车 SLAM 配置

本目录存放 ROS SLAM 小车的项目级 SLAM 配置：

- `cartographer/` —— **主 SLAM：Cartographer 2D 建图配置**（激光雷达为主，可选 IMU）
- `vi_car.yaml` / `cam0_pinhole.yaml` —— 备选 SLAM：VINS-Fusion 纯视觉单目配置

---

## 1. Cartographer（默认，激光雷达）

配置文件：`cartographer/vi_car_2d.lua`

### 数据链路

```
lidar_node(/scan) ─┐
imu_node(/imu/data)├─> cartographer_node(读取 vi_car_2d.lua) -> /map + /tf
camera_node(/camera/image_raw)   ← 保留接口，Cartographer 不消费
```

### 坐标系

`map`(全局) -> `odom`(cartographer 发布) -> `base_link` -> `laser_link` / `camera_link` / `imu_link`
（静态 TF 来自 `urdf/vi_car.urdf`，经 `robot_state_publisher` 发布）

### 启动方式

```bash
cd /home/lm/Ubuntu/code/ros_slam_car
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 一键启动 Cartographer（sim 雷达 + sim IMU + 相机）
ros2 launch launch/cartographer.launch.py
```

### 配置要点（vi_car_2d.lua）

| 项 | 值 | 说明 |
|----|----|------|
| `map_frame` / `odom_frame` | map / odom | TF 坐标系 |
| `tracking_frame` / `published_frame` | base_link | cartographer 估计并发布 odom->base_link |
| `provide_odom_frame` | true | 纯雷达 SLAM：由 cartographer 提供 odom |
| `num_laser_scans` | 1 | 订阅 `/scan` |
| `use_imu_data` | true | 使用 IMU（launch 里 `/imu` remap 到 `/imu/data`）；无 IMU 改 false |
| `min_range` / `max_range` | 0.15 / 12.0 | 按雷达实际量程调整（RPLIDAR A1 等） |
| `use_odometry` | false | 接入轮式里程计后改 true |

---

## 2. VINS-Fusion（备选，纯视觉单目）

配置文件：`vi_car.yaml` + `cam0_pinhole.yaml`

### 数据链路

```
FPV 图传(USB接收=UVC摄像头)
        │  src/hardware/camera/camera_node.cpp
        ▼
/camera/image_raw  (sensor_msgs/Image, bgr8, 640x480)
        │
        ▼
vins_node (src/slam/VINS-Fusion-ROS2-humble/vins)
        │  读取本目录 vi_car.yaml
        ▼
/odometry /path /camera_pose ... (纯视觉 VIO 输出)
```

### 启动方式

```bash
# 相机 + VINS（备选）
ros2 launch launch/vi_car.launch.py

# 另开终端可视化
ros2 launch vins vins_rviz.launch.xml
```

### ⚠️ 标定提醒

- `cam0_pinhole.yaml` 目前是**占位标定**（640x480，近似无畸变），不是真实相机参数。
- 请用 `camera_models` 包自带的 `intrinsic_calib` 对实际 FPV 相机重新标定，
  标定结果（含真实 fx/fy/cx/cy 和畸变系数）替换本文件后，链路才会定位准确。
- 标定时棋盘格图像尺寸应使用与相机节点一致的 640x480。

### 配置说明（vi_car.yaml）

| 键 | 值 | 说明 |
|----|----|------|
| `imu` | 0 | 纯视觉模式；接入 IMU 后改 1 并补 `/imu/data` 数据源 |
| `num_of_cam` | 1 | 单目 |
| `image0_topic` | `/camera/image_raw` | 与 camera_node 发布话题一致 |
| `cam0_calib` | `cam0_pinhole.yaml` | 标定文件（与本 yaml 同目录） |
| `image_width/height` | 640 / 480 | 必须与相机节点输出一致 |
| `estimate_extrinsic` / `body_T_cam0` | 0 / 占位矩阵 | 纯视觉模式不使用，但上游代码会无条件解析，缺失会启动崩溃，保留占位 |
| `world/body/camera_frame_id` | world / body / camera_link | TF 坐标系；camera_link 与相机节点图像 header 一致 |
| `output_path` | `<repo>/data/vins_output/` | 绝对路径，项目目录移动时需同步修改 |
