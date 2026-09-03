# lidar 激光雷达模块

本功能包负责激光雷达数据采集，发布标准 2D 激光话题 `/scan`（`sensor_msgs/LaserScan`），
作为 Cartographer 2D SLAM 的主传感器输入。

## 数据源

| 模式 | 参数 `lidar_source` | 说明 |
|---|---|---|
| `sim`（默认） | `"sim"` | 模拟模式，无需硬件生成一帧静态房间扫描（6m×6m 四壁 + 噪声），用于无雷达时联调 SLAM 链路 |
| `serial` | `"serial"` | 串口模式，真雷达（如 RPLIDAR A1）驱动**骨架**，协议解析待实现（见 `src/lidar_node.cpp` 中 TODO） |

## 使用方法

```bash
# 单独启动雷达节点
ros2 launch lidar lidar_launch.py

# 查看话题
ros2 topic echo /scan
```

## 参数（config/lidar_params.yaml）

| 参数 | 默认值 | 说明 |
|---|---|---|
| `lidar_source` | `sim` | `sim` 或 `serial` |
| `frame_id` | `laser_link` | 输出坐标系，与 urdf 一致 |
| `scan_fps` | 10 | 发布频率（Hz） |
| `range_min` / `range_max` | 0.15 / 12.0 | 量程，与 cartographer lua 配置对应 |
| `serial_port` / `baud_rate` | `/dev/ttyUSB0` / 115200 | 串口模式参数 |

## 目录结构

```
lidar/
├── config/lidar_params.yaml  # 参数
├── launch/lidar_launch.py    # 启动脚本
├── src/lidar_node.cpp        # 节点主程序
├── CMakeLists.txt
├── package.xml
└── README.md
```
