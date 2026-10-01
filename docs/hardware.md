# hardware 硬件驱动层

`src/drivers/` 用于统一管理所有硬件相关的 ROS 2 功能包，存放小车所有硬件设备的驱动和数据采集节点，
包括相机、IMU、串口通信、激光雷达等硬件模块。

## 目录结构
- `camera/`: 相机功能包，支持 USB 摄像头和 FPV 无线图传
- `imu/imu_ros2_device/`: 真实 IMU 驱动链（YbImu + madgwick 滤波，vendor 厂商 YbImuLib）
- `lidar/`:
  - `ydlidar_ros2_driver/`: YDLidar X3 ROS2 驱动（vendor 厂商例程 + Humble 适配）
  - `ydlidar_sdk/`: YDLidar-SDK 1.0.6（vendor 厂商源码，补 package.xml 作 colcon cmake 包）

规划中的 `serial/`（与下位机串口通信）见 [roadmap.md](roadmap.md)。

## 设计原则
1.  每个硬件模块都是独立的 ROS 2 包，职责单一
2.  所有硬件采集节点统一管理，便于维护和扩展
3.  硬件模块仅负责原始数据采集和标准话题发布，不包含算法处理逻辑
4.  厂商驱动源码原样 vendor（最小改动适配），入口统一走 `launch/include/hardware_sensors.launch.py`

## 数据链路（SLAM 用）
```
YbImu(/dev/myimu)  ─ ybimu_driver ─> /imu/data_raw ─ madgwick ─> /imu/data ─┐
YDLidar X3(/dev/ydlidar) ─ ydlidar_ros2_driver ────────────> /scan ────────┴─> cartographer
```

启动（需已插硬件并装好 udev 规则）：
```bash
sudo bash src/drivers/lidar/ydlidar_ros2_driver/startup/initenv.sh   # X3 -> /dev/ydlidar
ros2 launch launch/cartographer.launch.py use_imu:=true use_lidar:=true
```

串口固定：IMU udev -> `/dev/myimu`，雷达 udev -> `/dev/ydlidar`（见各包 README）。
