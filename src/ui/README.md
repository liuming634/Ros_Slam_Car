# ui 用户界面模块

本功能包负责ROS SLAM小车的用户界面，包括RViz可视化配置、WebUI界面等，用于实时显示小车状态、传感器数据、地图信息等，方便用户操作和监控小车。

## 功能特性
- RViz可视化：预设RViz配置文件，快速加载机器人模型、传感器数据、地图等
- 实时状态监控：显示小车位姿、传感器数据、电池状态等信息
- 远程控制：通过WebUI或其他界面方式远程控制小车运动
- 地图管理：地图的显示、保存、加载等操作

## 目录结构
```
ui/
├── config/                # 配置文件目录
│   └── rviz_config.rviz     # RViz配置文件
├── include/ui/            # 头文件目录
├── launch/                # 启动文件目录
│   └── ui_launch.py      # UI启动脚本
├── src/                   # 源代码目录
│   └── ui_node.cpp       # UI节点主程序
├── CMakeLists.txt         # 编译配置
├── package.xml            # 包依赖声明
└── README.md              # 本说明文档
```

## 使用说明
1.  确保其他功能节点正常运行
2.  直接启动RViz：`rviz2 -d config/rviz_config.rviz`
3.  或启动UI节点：`ros2 launch ui ui_launch.py`
4.  在RViz中添加需要的显示插件，监控小车状态