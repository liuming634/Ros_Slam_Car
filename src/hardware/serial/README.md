# serial 串口通信模块

本功能包负责与下位机进行串口通信，实现PC上位机与嵌入式控制器之间的数据传输，比如发送控制指令、接收传感器数据等。

## 功能特性
- 支持标准串口通信协议
- 支持自定义波特率、数据位、停止位、奇偶校验等参数
- 内置数据解析和校验功能，确保通信可靠性
- 标准ROS2话题发布/订阅：`/serial/data`（自定义消息）和`/serial/cmd`
- 支持多种串口设备（USB转串口、蓝牙串口等）

## 使用方法
1.  将串口设备连接到PC
2.  修改config/serial_params.yaml配置文件，设置串口端口和参数
3.  编译并启动节点：`ros2 launch serial serial_launch.py`
4.  发送控制指令：`ros2 topic pub /serial/cmd std_msgs/msg/String "data: 'your_command'"`
5.  查看接收数据：`ros2 topic echo /serial/data`

## 目录结构
```
serial/
├── config/                # 配置文件目录
│   └── serial_params.yaml   # 串口参数配置
├── include/serial/        # 头文件目录
├── launch/                # 启动文件目录
│   └── serial_launch.py   # 启动脚本
├── src/                   # 源代码目录
│   └── serial_node.cpp   # 串口节点主程序
├── CMakeLists.txt         # 编译配置
├── package.xml            # 包依赖声明
└── README.md              # 本说明文档
```