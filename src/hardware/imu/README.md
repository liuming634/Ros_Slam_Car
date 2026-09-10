# imu IMU惯性测量单元模块

本功能包负责IMU惯性测量单元的数据采集和话题发布，为SLAM（Cartographer / VINS）提供运动姿态数据。
标准话题：`/imu/data`（`sensor_msgs/msg/Imu`）和 `/imu/temperature`（`sensor_msgs/msg/Temperature`）。

## 数据源

| 模式 | 参数 `imu_source` | 说明 |
|---|---|---|
| `sim` | `"sim"` | 模拟模式，无需硬件发布静止 IMU 读数（重力竖直向上 9.81），用于无 IMU 时联调链路 |
| `hardware` | `"hardware"` | Yahboom YbImu 真实硬件链（下方 `imu_ros2_device` 子包，已适配） |

`launch/cartographer.launch.py` 的 IMU 链由 `imu_source` / `use_imu` 控制：

```bash
# 默认 sim（无硬件跑通链路）
ros2 launch launch/cartographer.launch.py

# 插上 YbImu 硬件后（串口自动扫描 /dev/myimu、/dev/ttyUSB0-2，波特率 115200）
ros2 launch launch/cartographer.launch.py imu_source:=hardware

# 数据链路：ybimu_driver(/imu/data_raw, /imu/mag, /baro, /euler)
#           -> imu_filter_madgwick(/imu/data) -> Cartographer
ros2 launch imu_ros2_device ybimu_display.launch.py   # 单独测试驱动 + rviz
```

驱动包 `imu_ros2_device` 位于 `src/hardware/imu_ros2_device`（ament_python，
基于厂商 YbImuLib 驱动库适配，已 vendor 在包内）：
- 加速度按 ROS 标准由 g 转为 m/s²（静止重力幅值约 9.81）
- madgwick 滤波输出 frame 定为 `imu_link`（与 urdf 一致，Cartographer tracking_frame 依赖）

## 使用方法

```bash
# 单独启动 IMU 节点
ros2 launch imu imu_launch.py

# 查看话题
ros2 topic echo /imu/data
```

## 参数（config/imu_params.yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `imu_source` | `sim` | `sim` 或 `serial` |
| `frame_id` | `imu_link` | 输出坐标系，与 urdf 一致 |
| `publish_rate` | 50 | 发布频率（Hz） |
| `serial_port` / `baud_rate` | `/dev/ttyUSB1` / 115200 | 串口模式参数 |

## 目录结构

```
imu/
├── config/imu_params.yaml  # 参数
├── launch/imu_launch.py    # 启动脚本
├── src/imu_node.cpp        # 节点主程序
├── CMakeLists.txt
├── package.xml
└── README.md
```
