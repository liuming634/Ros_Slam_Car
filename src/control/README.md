# control 控制模块

本功能包负责ROS SLAM小车的运动控制，包括底盘控制、PID速度控制、路径跟踪等功能，实现将SLAM输出的位姿数据转换为小车的运动指令。

## 功能特性
- 底盘运动控制：支持差速、阿克曼等多种底盘结构
- PID速度控制：精确控制电机转速和运动速度
- 路径跟踪：支持纯追踪、Stanley等多种路径跟踪算法
- 控制指令发布：将运动控制指令发送到底盘驱动节点
- 支持ROS2标准控制话题：`/cmd_vel`（geometry_msgs/msg/Twist）

## 目录结构
```
control/
├── config/                # 配置文件目录
│   └── control_params.yaml # 控制参数配置
├── include/control/        # 头文件目录
├── launch/                # 启动文件目录
│   └── control_launch.py  # 启动脚本
├── src/                   # 源代码目录
│   └── control_node.cpp  # 控制节点主程序
├── CMakeLists.txt         # 编译配置
├── package.xml            # 包依赖声明
└── README.md              # 本说明文档
```

## 使用方法
1.  确保底盘驱动节点正常运行
2.  修改config/control_params.yaml配置文件，设置底盘参数和控制参数
3.  编译并启动控制节点：`ros2 launch control control_launch.py`
4.  发送运动指令：`ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"`