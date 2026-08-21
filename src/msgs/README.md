# msgs 自定义消息包

本功能包存放ROS SLAM小车项目的自定义ROS消息、服务和动作定义，用于在不同功能包之间传递自定义数据格式。

## 功能特性
- 自定义消息类型：用于传递项目特定的传感器数据、控制指令等
- 自定义服务类型：用于实现ROS服务调用
- 自定义动作类型：用于长时间运行的任务（如路径规划）

## 目录结构
```
msgs/
├── msg/                   # 消息定义目录
├── srv/                   # 服务定义目录
├── action/                # 动作定义目录
├── CMakeLists.txt         # 编译配置
├── package.xml            # 包依赖声明
└── README.md              # 本说明文档
```

## 使用说明
1.  在项目根目录的CMakeLists.txt和package.xml中添加对本包的依赖
2.  编译后即可在其他包中使用自定义消息类型

## 示例
例如，如果你需要定义一个自定义的小车状态消息，可以在msg目录下创建`CarState.msg文件，内容如下：
```
std_msgs/Header header
float32 linear_x
float32 angular_z
float32 battery_voltage
```