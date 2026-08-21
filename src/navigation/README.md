# navigation 导航模块

本功能包负责ROS SLAM小车的自主导航功能，包括全局路径规划、局部路径规划、避障等功能，实现从当前位置到目标位置的自主导航。

## 功能特性
- 全局路径规划：基于A*、Dijkstra等算法的全局路径规划
- 局部路径规划：基于TEB、DWA等算法的局部路径规划
- 动态避障：支持激光雷达、相机等传感器的实时避障
- 目标点导航：支持通过ROS话题或RViz设置导航目标点
- 兼容ROS2标准导航栈

## 目录结构
```
navigation/
├── config/                # 配置文件目录
│   ├── nav2_params.yaml    # Nav2导航参数配置
│   └── costmap_common_params.yaml # 代价地图参数
├── launch/                # 启动文件目录
│   └── navigation_launch.py # 导航启动脚本
├── src/                   # 源代码目录
│   └── navigation_node.cpp # 导航节点主程序
├── CMakeLists.txt         # 编译配置
├── package.xml            # 包依赖声明
└── README.md              # 本说明文档
```

## 使用方法
1.  确保SLAM节点正常运行，发布`/map`和`/odom`话题
2.  确保底盘控制节点正常运行，订阅`/cmd_vel`话题
3.  修改config目录下的导航参数配置文件
4.  编译并启动导航节点：`ros2 launch navigation navigation_launch.py`
5.  在RViz中设置导航目标点，小车将自动导航到目标位置