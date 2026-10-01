#  ROS SLAM小车上位机相机功能包（基于FPV图传）

本功能包为ROS SLAM小车项目提供基于FPV无线图传的视频流处理功能，同时兼容USB摄像头模式。我们使用FPV图传作为主要的图像输入设备，在PC上位机上完成所有图像处理和SLAM算法的运行，实现低成本、远距离的第一视角图传和建图功能。

## 功能特性

- **FPV图传优先支持**：以FPV无线图传作为核心图像输入方案，完美适配ROS SLAM小车的第一视角图传需求，支持RTL8812AU WiFi适配器和WFB-ng协议
- **USB摄像头兼容模式**：同时支持USB摄像头作为备用输入，方便本地调试
- **ROS2标准话题发布**：将原始相机图像发布到`camera/image_raw`标准话题，兼容所有ROS2 SLAM算法
- **上位机集中处理**：所有图像处理和算法运行都在PC上位机完成，无需在嵌入式设备上部署复杂软件
- **可配置参数**：所有参数均可通过YAML配置文件灵活调整，支持FPV信道、编码格式、分辨率等定制
- **图像预处理**：支持水平/垂直翻转、分辨率调整、帧率控制等图像增强功能
- **FPV视频解密**：内置FPV视频流解密功能，支持使用自定义密钥文件进行解密

## 依赖环境

- ROS2 Humble/Iron/Jazzy 及以上版本
- OpenCV 4.x 图像处理库
- cv_bridge ROS桥接库
- libusb-1.0 USB设备开发库
- libsodium 加密库
- FFmpeg 音视频编解码库
- ament_cmake ROS编译系统
- rclcpp ROS2 C++客户端库
- sensor_msgs ROS标准传感器消息

## 安装步骤

1. 将本功能包放置到你的ROS2工作空间的`src`目录下

2. 安装系统依赖包：
   ```bash
   sudo apt install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-sensor-msgs libopencv-dev libusb-1.0-0-dev libsodium-dev libavcodec-dev libavformat-dev libswscale-dev
   ```

3. 编译功能包：
   ```bash
   colcon build --packages-select camera
   ```

## 配置说明

所有参数均可在`config/camera_params.yaml`配置文件中调整：

### 通用相机参数

| 参数名称 | 类型 | 默认值 | 参数说明 |
|-----------|------|---------|-------------|
| `camera_source` | 字符串 | `usb` | 相机源类型：`usb`(USB摄像头) 或 `fpv`(FPV图传) |
| `camera_id` | 整数 | 0 | USB摄像头设备ID，通常`/dev/video0`对应0 |
| `camera_width` | 整数 | 1920 | 相机输出图像宽度 |
| `camera_height` | 整数 | 1080 | 相机输出图像高度 |
| `camera_fps` | 整数 | 30 | 相机采集帧率 |
| `camera_hflip` | 布尔值 | false | 是否启用水平翻转 |
| `camera_vflip` | 布尔值 | false | 是否启用垂直翻转 |
| `camera_matrix_file` | 字符串 | `calibration_data/camera_matrix.npy` | 相机标定内参文件路径 |
| `dist_coeffs_file` | 字符串 | `calibration_data/dist_coeffs.npy` | 相机畸变系数文件路径 |

### FPV专用参数

| 参数名称 | 类型 | 默认值 | 参数说明 |
|-----------|------|---------|-------------|
| `fpv_key` | 字符串 | `gs.key` | FPV视频解密密钥文件路径 |
| `fpv_channel` | 整数 | 161 | FPV图传使用的WiFi信道 |
| `fpv_channel_width` | 整数 | 20 | WiFi信道带宽：20/40/80 MHz |
| `fpv_codec` | 字符串 | `h265` | 视频编码格式：`h264` 或 `h265` |
| `fpv_width` | 整数 | 1920 | FPV图像输出宽度 |
| `fpv_height` | 整数 | 1080 | FPV图像输出高度 |
| `fpv_fps` | 整数 | 60 | FPV视频采集帧率 |

## 使用方法

> 注意：本功能包运行在PC上位机上，需要连接FPV图传接收器或USB摄像头，无需在嵌入式设备上编译。

### 使用默认参数启动

```bash
ros2 launch camera camera_launch.py
```

### 使用自定义配置文件启动

```bash
ros2 launch camera camera_launch.py params_file:=/path/to/your/custom_config.yaml
```

### 验证图像话题发布

1. 查看当前活跃话题：
   ```bash
   ros2 topic list
   ```
   你将在输出结果中看到 `/camera/image_raw` 话题

2. 查看图像原始数据：
   ```bash
   ros2 topic echo /camera/image_raw
   ```

3. 使用RViz可视化图像：
   ```bash
   rviz2
   ```
   启动RViz后添加`Image`显示插件，并将话题设置为 `/camera/image_raw` 即可看到实时画面

## FPV模式配置指南（上位机使用）

1. 准备RTL8812AU型号的WiFi适配器，并将其连接到PC上位机
2. 从FPV发射端获取解密密钥文件`gs.key`
3. 将`gs.key`复制到功能包根目录，或在配置文件中修改密钥文件路径
4. 在配置文件中将`camera_source`设置为`fpv`
5. 正常启动节点即可，所有FPV视频流的接收、解密和图像处理都将在上位机上完成，无需在嵌入式设备上运行任何软件

## 目录结构

```
camera/
├── config/                # 配置文件目录
│   └── camera_params.yaml # 相机参数配置文件
├── fpv/                   # FPV图传功能模块
│   ├── include/           # FPV相关头文件
│   └── src/               # FPV图传实现代码（RTL8812AU驱动、WFB协议解析等）
├── include/               # 自定义头文件目录（当前为空）
├── launch/                # ROS启动文件目录
│   └── camera_launch.py  # 相机节点启动脚本
├── src/                   # 源代码目录
│   └── camera_node.cpp   # 相机节点主程序入口
├── CMakeLists.txt         # CMake编译配置文件
├── package.xml            # ROS包依赖声明文件
└── README.md              # 本说明文档
```

## 常见问题排查

### 无法打开相机设备
- 检查相机是否正确连接：执行 `ls /dev/video*` 查看设备列表
- 确认配置文件中的`camera_id`参数与实际设备匹配
- 添加用户到视频用户组：`sudo usermod -aG video $USER`，注销后生效

### FPV图传无法接收
- 检查RTL8812AU适配器是否识别：执行 `lsusb` 应显示`0bda:8812`设备
- 确认WiFi信道与FPV发射端一致
- 检查解密密钥文件路径是否正确
- 确保FPV发射端已上电开机
