# 功能规划（待实现模块）

本文件记录**规划中但尚未实现**的模块。目录不会预先占位——需要时在对应分层下新建
ROS 包，`colcon` 会自动发现。

## 分层归属对照

| 规划模块 | 归属分层 | 目标目录 | 职责 |
|---|---|---|---|
| 串口通信 | `drivers` | `src/drivers/serial/` | 与下位机串口通信，收发传感器数据与控制指令 |
| 自定义消息 | `interfaces` | `src/interfaces/` | 自定义 msg / srv / action 定义 |
| 感知 | `algorithms` | `src/algorithms/perception/` | 目标检测、跟踪、语义分割、点云处理 |
| 控制 | `algorithms` | `src/algorithms/control/` | 底盘控制、PID 速度环、路径跟踪 |
| 导航 | `algorithms` | `src/algorithms/navigation/` | 全局/局部路径规划、动态避障 |
| 通用工具 | 共享 | `src/common/` | 坐标/四元数转换、滤波、ROS 通信封装 |
| 界面 | 共享 | `src/ui/` | RViz 预设配置、状态监控、远程控制 |

## 各模块要点

### drivers/serial —— 串口通信

与下位机（嵌入式控制器）通信，透传控制指令与传感器数据。

- 话题：`/serial/data`（上行）、`/serial/cmd`（下行，`std_msgs/msg/String`）
- 参数化波特率、数据位、停止位、奇偶校验
- 多串口设备支持（USB 转串口、蓝牙串口）

### interfaces —— 自定义消息

跨包传递项目特定格式：传感器数据、控制指令、规划结果。
长期运行的规划任务走 action，不用 srv。

### algorithms/perception —— 感知

SLAM 与导航的环境理解层：目标检测（YOLO/SSD）、多目标跟踪、语义分割、
激光点云滤波/分割/配准、障碍物与交通标志识别。
**相机原始图像驱动已在 `drivers/camera`，本模块指其上的算法。**

### algorithms/control —— 控制

把路径或 SLAM 位姿转成电机指令：差速/阿克曼底盘适配、PID 速度环、
纯追踪 / Stanley 路径跟踪，输出标准 `/cmd_vel`（`geometry_msgs/msg/Twist`）
给底盘驱动节点。

### algorithms/navigation —— 导航

目标点 → 运动路径：全局规划（A*/Dijkstra）、局部规划（TEB/DWA）、
雷达/相机实时避障，兼容 Nav2 标准导航栈。
**前置依赖：Cartographer 存图 → AMCL 定位。**

### common —— 通用工具

跨包共享的工具函数与数据结构：坐标变换、欧拉角/四元数互转、向量矩阵运算、
滤波算法、ROS 通信封装。目的是消除重复代码。

### ui —— 用户界面

RViz 预设配置（快速加载机器人模型 / 传感器 / 地图）、实时状态监控
（位姿、传感器、电池）、地图显示保存加载、远程控制。
**RViz 配置文件的落点见 `src/robot/vi_car_description/rviz/`。**

## 规划路线

```
① Cartographer 建图链路（已就绪）
② 接入真实传感器（lidar/imu 串口驱动，已就绪）
③ 实车建图 + 保存地图
④ AMCL 地图内定位
⑤ Nav2 路径规划 + 底盘控制（需 interfaces + control + navigation）
⑥ 多源融合（IMU / 轮式里程计经 robot_localization）
```