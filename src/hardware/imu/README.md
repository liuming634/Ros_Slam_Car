# imu IMU惯性测量单元模块

本功能包负责IMU惯性测量单元的数据采集和ROS话题发布，为SLAM算法提供运动姿态数据。

## 功能特性
- 支持多种主流IMU传感器（如MPU6050、BNO085等）
- 标准ROS2话题发布：`/imu/data`（sensor_msgs/msg/Imu）和`/imu/temperature`
- 支持参数配置：采样率、量程、滤波参数等
- 数据校准：内置零偏校正、温度补偿等功能

## 使用方法
1.  将IMU传感器连接到PC或下位机
2.  修改config/imu_params.yaml配置文件，设置传感器类型和参数
3.  编译并启动节点：`ros2 launch imu imu_launch.py`
4.  查看话题：`ros2 topic echo /imu/data`

## 目录结构
```
imu/
├── config/                # 配置文件目录
│   └── imu_params.yaml    # IMU参数配置
├── include/imu/          # 头文件目录
├── launch/                # 启动文件目录
│   └── imu_launch.py     # 启动脚本
├── src/                   # 源代码目录
│   └── imu_node.cpp     # IMU节点主程序
├── CMakeLists.txt         # 编译配置
├── package.xml            # 包依赖声明
└── README.md              # 本说明文档
```