# vi_car - 小车 VINS-Fusion 配置

本目录存放 ROS SLAM 小车的 VINS-Fusion 项目级配置（纯视觉单目，当前未接 IMU）。

## 数据链路

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

## 启动方式

```bash
# 构建工作区（首次）
cd /home/lm/Ubuntu/code/ros_slam_car
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 一键启动 相机 + VINS
ros2 launch launch/vi_car.launch.py

# 另开终端可视化
ros2 launch vins vins_rviz.launch.xml
```

## ⚠️ 标定提醒

- `cam0_pinhole.yaml` 目前是**占位标定**（640x480，近似无畸变），不是真实相机参数。
- 请用 `camera_models` 包自带的 `intrinsic_calib` 对实际 FPV 相机重新标定，
  标定结果（含真实 fx/fy/cx/cy 和畸变系数）替换本文件后，链路才会定位准确。
- 标定时棋盘格图像尺寸应使用与相机节点一致的 640x480。

## 配置说明（vi_car.yaml）

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
