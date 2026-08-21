# perception 感知模块

本功能包负责除相机之外的其他感知任务，包括目标检测、目标跟踪、语义分割、点云处理等，为SLAM和导航提供环境感知信息。

## 功能特性
- 目标检测：支持YOLO、SSD等多种目标检测算法
- 目标跟踪：实现多目标跟踪功能
- 语义分割：实现场景语义分割
- 点云处理：激光雷达点云的滤波、分割、配准等
- 环境感知：识别障碍物、车道线、交通标志等环境信息

## 目录结构
```
perception/
├── config/                # 配置文件目录
│   └── perception_params.yaml # 感知参数配置
├── include/perception/    # 头文件目录
├── launch/                # 启动文件目录
│   └── perception_launch.py # 感知启动脚本
├── src/                   # 源代码目录
│   └── perception_node.cpp # 感知节点主程序
├── CMakeLists.txt         # 编译配置
├── package.xml            # 包依赖声明
└── README.md              # 本说明文档
```

## 使用说明
1.  确保传感器数据正常发布（相机图像、激光点云等）
2.  修改config/perception_params.yaml配置文件，设置感知算法参数
3.  编译并启动感知节点：`ros2 launch perception perception_launch.py`
4.  查看感知结果话题：`ros2 topic list | grep perception`