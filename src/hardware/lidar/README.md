# lidar 激光雷达模块

本功能包负责激光雷达数据采集，发布标准 2D 激光话题 `/scan`（`sensor_msgs/LaserScan`），
作为 Cartographer 2D SLAM 的主传感器输入。

## 数据源

`launch/cartographer.launch.py` 顶层统一用 `lidar_source` 控制：

```bash
# 默认 sim（无雷达跑通链路，占位包 src/hardware/lidar）
ros2 launch launch/cartographer.launch.py

# 插上 YDLidar X3 后
ros2 launch launch/cartographer.launch.py lidar_source:=hardware

# 数据链路：ydlidar_ros2_driver_node(/scan, frame_id=laser_link) -> Cartographer
```

真实驱动包 `ydlidar_ros2_driver` 位于 `src/hardware/ydlidar_ros2_driver`
（基于厂商例程适配，12 处 `declare_parameter` 补默认值适配 Humble），
依赖 `src/hardware/ydlidar_sdk`（vendor 的 YDLidar-SDK 1.0.6）。

| 模式 | 参数 | 说明 |
|---|---|---|
| `sim`（默认） | `"sim"` | 模拟模式，无需硬件生成一帧静态房间扫描（6m×6m 四壁 + 噪声），用于无雷达时联调 SLAM 链路 |
| `hardware` | `"hardware"` | YDLidar X3 真雷达（厂商 ydlidar_ros2_driver） |

### 硬件注意事项（X3）

1. **udev 规则**（X3 使用 CP210x USB 串口，映射成 `/dev/ydlidar`）：
   ```bash
   sudo bash src/hardware/ydlidar_ros2_driver/startup/initenv.sh  2>/dev/null || \
   echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ydlidar"' | \
     sudo tee /etc/udev/rules.d/ydlidar.rules
   udevadm control --reload && 重插雷达
   ```
2. 驱动参数 `params/ydlidar_x3.yaml`（X3：115200 单通道 3K 采样 10Hz 8m），
   `frame_id=laser_link` 与 `urdf/vi_car.urdf` 的 `base_link->laser_link` 对齐；
   TF 由 `robot_state_publisher` 统一发布，驱动自身不发 TF（与例程一致）。
3. **串口冲突提醒**：`/dev/ttyUSB*` 同时会被 IMU 驱动（`/dev/myimu, ttyUSB0-2`）扫描。
   装好 udev 规则后雷达固定为 `/dev/ydlidar`、IMU 固定为 `/dev/myimu`，互不干扰。

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
