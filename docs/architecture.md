# 分层架构

## 分层原则

项目代码只分三层，依赖方向**单向向下**，不允许跨层反向引用：

```
┌─────────────────────────────────────────────────────────┐
│ ③ algorithms/          算法层                            │
│    消费 ROS 话题 → 产出 map / odom / tf                  │
│    cartographer_ros/  vins_fusion/                       │
├─────────────────────────────────────────────────────────┤
│ ① drivers/             驱动层                            │
│    读硬件 → 发 ROS 话题                                 │
│    lidar/  imu/  camera/                                 │
└─────────────────────────────────────────────────────────┘
┌─────────────────────────────────────────────────────────┐
│ ② robot/               本体描述层                        │
│    坐标系唯一真源，不含逻辑                              │
│    vi_car_description/                                   │
└─────────────────────────────────────────────────────────┘
```

- **驱动层不依赖算法层**：驱动只管把传感器数据发成标准话题，不知道谁在消费。
- **算法层不直接碰硬件**：只订阅话题，因此仿真（Gazebo 插件）与真实硬件可以无缝替换。
- **本体描述层被所有人依赖，但自己不含逻辑**：静态 TF 的唯一来源。

## 话题与坐标系数据流

```
仿真（use_sim:=true）                硬件（use_sim:=false）
─────────────────────                ─────────────────────
Gazebo 传感器插件                     YDLidar X3 驱动
  /scan      frame=laser_link          → /scan      frame=laser_link
  /imu/data  frame=imu_link             YbImu 驱动 → Madgwick
Gazebo 差速插件                        → /imu/data  frame=imu_link
  /odom     odom->base_footprint       USB/FPV 相机
  /clock                              → /camera/image_raw
         │                                      │
         └──────────────┬───────────────────────┘
                        ▼
              cartographer_node
                        │
        ┌───────────────┼───────────────┐
        ▼               ▼               ▼
    /map 栅格      map->odom->      /tf_static
    (occupancy_    base_link        base_link->laser_link
     grid_node)                     base_link->camera_link
                                    base_link->imu_link
                                        ▲
                     robot_state_publisher（读 vi_car_description 的 URDF）
```

两条链路的话题名与 `frame_id` **完全一致**，这是仿真/硬件可切换的前提。

## 目录职责

| 目录 | 职责 | 放什么 | 不放什么 |
|---|---|---|---|
| `src/drivers/` | 硬件抽象 | 串口/USB 协议解析、厂商 SDK | 任何算法逻辑 |
| `src/algorithms/` | 感知与估计 | SLAM、VIO、检测、规划、控制 | 硬件寄存器操作 |
| `src/robot/` | 本体建模 | URDF、world、RViz 配置 | 节点代码、参数调优 |
| `config/` | 项目级调参 | 跨包的算法配置（lua / yaml） | 包内私有参数 |
| `launch/` | 场景编排 | 节点启停与连线 | 算法参数（应在 config/） |
| `assets/` | 产物 | 地图、轨迹输出 | 源码与配置 |

### config/ 与包内 config/ 的分工

参数放哪，取决于**谁拥有它**：

- **包内** `src/drivers/camera/config/camera_params.yaml` —— 只影响 camera 一个包，由包自己拥有，随包安装。
- **顶层** `config/cartographer/vi_car_2d.lua` —— 对 vendored 上游 Cartographer 的项目级调参。上游默认值不适合本车，但改的是上游文件，所以放顶层，不污染 vendored 源码。

判断标准：**改这个参数是否需要动上游源码？** 需要 → 顶层 `config/`；不需要 → 包内。

## 命名约定

ROS 2 官方风格：包名、功能节点、话题一律 **snake_case**；Python 包、launch 目录用 snake_case。
目录分组（`drivers/`、`algorithms/`）是本项目的组织约定，不是 ROS 规范——
`colcon` 递归扫描 `src/`，分组层级不影响构建，纯粹为了人读起来清楚。

## 新增模块时

1. 判断它属于哪一层（驱动 / 算法 / 本体）。
2. 在该层下新建包目录，不要预先创建空的分类目录。
3. 若需要跨包通信的自定义消息，在 `src/interfaces/` 下建消息包（尚未创建）。
4. 顶层 `config/` 只放跨包调参；包私有参数留在包内。
5. 更新本文档的表格与数据流图。

规划中的模块见 [roadmap.md](roadmap.md)。