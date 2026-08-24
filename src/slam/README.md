# slam SLAM模块

本功能包负责ROS SLAM小车的同时定位与建图功能，集成主流的SLAM算法（如VINS-Fusion、Cartographer等），实现基于传感器数据的机器人位姿估计和环境地图构建。

## 功能特性
- 支持多种主流SLAM算法：视觉SLAM、激光SLAM、多传感器融合SLAM
- 实时位姿估计：输出机器人的实时位姿信息
- 环境地图构建：构建2D/3D环境地图
- 地图保存与加载：支持地图的保存和加载功能
- 兼容ROS2标准SLAM话题：`/odom`、`/map`、`/tf`等

## 目录结构
```
slam/
├── config/                # 配置文件目录
│   ├── slam_params.yaml    # SLAM算法参数配置
│   └── rtabmap_params.yaml # RTAB-Map参数配置（可选）
├── launch/                # 启动文件目录
│   └── slam_launch.py     # SLAM启动脚本
├── src/                   # 源代码目录
│   └── slam_node.cpp     # SLAM节点主程序
├── CMakeLists.txt         # 编译配置
├── package.xml            # 包依赖声明
└── README.md              # 本说明文档
```

## 使用说明
1.  确保传感器预处理节点正常发布预处理后的传感器数据
2.  修改config/slam_params.yaml配置文件，设置SLAM算法参数
3.  编译并启动SLAM节点：`ros2 launch slam slam_launch.py`
4.  查看位姿和地图话题：`ros2 topic list | grep odom map`
## VINS-Fusion 源码来源

- `VINS-Fusion-ROS2-humble/` 为 vendored 代码（无嵌套 .git），上游：https://github.com/zinuok/VINS-Fusion-ROS2
- 小车自身的 VINS 配置（纯视觉单目）在顶层 `config/vi_car/`，不随包内源码管理
