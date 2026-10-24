# Nav2 自主导航

用 Nav2 实现"小车在已知地图上自主行驶到指定目标点"。建图部分仍由 Cartographer 负责，
本文只讲**定位 + 规划 + 控制**这一段。

---

## 1. 整体数据流

```
  assets/maps/vi_car.yaml
          │
          ▼  map_server 读取并发布
      /map（静态栅格图，发布后不变）
          │
          ├──► AMCL ◄── /scan（雷达）      AMCL = 粒子滤波定位
          │       ▲                       职责：拿雷达和地图比对，算出"我在哪"
          │       └── /odom（轮式里程计）  职责：预测"车大概移动了多少"
          │       │
          │       └───► TF: map → odom     ★ AMCL 是这条 TF 的唯一写入者
          │
          ▼
    Nav2（7 个节点）
      ├─ global_costmap  全局代价地图（底图 + 实时障碍，frame=map）
      ├─ local_costmap   局部代价地图（滚动窗口，frame=odom）
      ├─ planner_server  全局规划：算出绕开障碍的路径 → /plan
      ├─ controller_server 局部避障 + 跟踪 → /cmd_vel_nav
      ├─ velocity_smoother 速度平滑 → /cmd_vel
      ├─ bt_navigator  行为树：调度"规划→控制→失败重试"
      ├─ behavior_server / smoother_server / waypoint_follower
          │
          ▼
     /cmd_vel（geometry_msgs/msg/Twist）
          │
          ▼  Gazebo 差速插件订阅，内部换算成轮速
      四个轮子转动 → 小车行驶
```

**关键认知：Nav2 完全不知道小车有几个轮子、是不是差速。** 它只输出
`/cmd_vel` 里的两个有效数字：

| 字段 | 含义 | 单位 |
|---|---|---|
| `linear.x` | 前后速度 | m/s |
| `angular.z` | 转向角速度（正=左转，负=右转） | rad/s |

轮子怎么转是底盘的事。你的 `libgazebo_ros_diff_drive.so` 已经订阅了 `/cmd_vel`，
**无需写任何代码即可对接。**

---

## 2. 两条链路互斥（为什么需要两份 URDF）

`map → odom` 这条 TF 在整个系统里**只能有一个写入者**。建图和导航需要由不同节点承担这个角色：

| 环节 | 建图模式 | 导航模式 |
|---|---|---|
| 谁写 `map → odom` | Cartographer（位姿图优化算出来） | AMCL（粒子滤波算出来） |
| 谁写 `odom → base_footprint` | 无（Cartographer 直接输出到 `base_link`） | Gazebo 差速插件（轮子转了多少） |
| 启动入口 | `launch/simulation.launch.py` | `launch/navigation.launch.py` |
| 用的 URDF | `vi_car.urdf`（`publish_odom_tf=false`） | `vi_car_nav.urdf`（`publish_odom_tf=true`） |

**不要同时启动这两个 launch**，会争抢 TF。

> 为什么建图时不需要 `odom → base_footprint`？因为 Cartographer 的
> `published_frame = "base_link"`，它直接输出 `odom → base_link`，
> 中间不需要轮式里程计插一脚。

---

## 3. 文件清单

```
config/nav2/vi_car_nav2_params.yaml          新建  Nav2 参数（官方模板 + 本车改动）
launch/navigation.launch.py                  新建  导航模式启动入口
launch/include/robot_state_publisher.launch.py       改动  新增 urdf_file 参数
src/robot/vi_car_description/urdf/
    vi_car.urdf                                      未改动（建图用）
    vi_car_nav.urdf                          新建  导航用，与上者仅差 publish_odom_tf
docs/navigation.md                            新建  本文档
```

---

## 4. 参数文件改了什么

官方模板：`/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml`
（这是 ROS 2 Humble 官方 TurtleBot3 Waffle 的配置，**本机 Nav2 已通过 apt 预装，
全部是二进制 `.so`，没有 C++ 源码，不需要编译源码**）

文件里每处改动都标了 `# [#]` 注释，共 6 类：

### ① `robot_base_frame`：`base_link` → `base_footprint`（4 处）

| 位置 | 参数 |
|---|---|
| `amcl.base_frame_id` | 官方本就是 `base_footprint`（未改） |
| `bt_navigator.robot_base_frame` | `base_link` → `base_footprint` |
| `global_costmap.robot_base_frame` | `base_link` → `base_footprint` |
| `local_costmap.robot_base_frame` | `base_link` → `base_footprint` |

**原因**：Gazebo 差速插件的 `<robot_base_frame>` 是 `base_footprint`
（因为轮轴高度不是 0，`base_footprint` 才是地面投影原点）。若 Nav2 配置成
`base_link` 而 Gazebo 发的是 `odom → base_footprint`，AMCL 会持续报：

```
Could not transform from odom to base_footprint
```

### ② `footprint`：替代 `robot_radius: 0.22`

```yaml
# 官方（TurtleBot3 是接近圆形的车，用圆近似误差小）
robot_radius: 0.22

# 本车：底盘 0.25 × 0.10 m 的长方形，用多边形精确描述
footprint: "[[-0.125,-0.05],[-0.125,0.05],[0.125,0.05],[0.125,-0.05]]"
footprint_padding: 0.01
```

**为什么必须改（两个方向的误差都是致命的）：**

代价地图每帧会把机器人 footprint 覆盖的栅格标记为障碍。如果 footprint 和真实车身不一致：

| 情况 | 后果 |
|---|---|
| footprint 比车大 | 规划器认为"车会撞"而不敢走窄道，路径规划失败/反复重规划 |
| footprint 比车小 | 真实车身蹭到墙、柱子（Gazebo 物理碰撞），Nav2 却认为没碰撞 |

车体是 25cm 长条形，用半径 13.5cm 的圆去近似会让两侧各多占 8.5cm，
贴墙时容易规划失败。改用精确矩形后，代价地图上占的形状就是真实车身。

**这是 2D 栅格导航的固有约束**：栅格图是俯视的，没有高度概念，
Nav2 不会读 URDF 来推断车身形状，必须在参数里显式声明。

> **URDF 与 Nav2 参数必须手工同步。** 以后用 SolidWorks 改底盘尺寸后，
> 需要同步修改：URDF 的 `<box size>`、Nav2 的 `footprint`、
> 差速插件的 `wheel_separation` / `wheel_diameter`。Nav2 不会自动同步。

### ③ `obstacle_max_range` / `raytrace_max_range`

```yaml
# 官方（按 TurtleBot3 调的）
raytrace_max_range: 3.0
obstacle_max_range: 2.5

# 本车雷达量程 0.10 ~ 7.0 m
raytrace_max_range: 6.0
obstacle_max_range: 5.0
```

**含义**：`obstacle_max_range` 之外的障碍物，Nav2 **完全看不到**
（不写入代价地图）。如果沿用官方的 2.5m，而本车雷达能看 7m，
那么 3m 处的墙在 Nav2 眼里是不存在的 → 规划器会直接规划一条撞墙的路线。

**约束**：`obstacle_max_range` 必须 ≤ 雷达真实量程。填比雷达量程还大的值
会产生"幽灵障碍"（Nav2 认为很远有障碍，但实际传感器根本没测到），导致机器人无谓绕路。

> `global_costmap` 和 `local_costmap` 两处都要改，它们是独立的配置块。

### ④ `amcl.laser_max_range` / `laser_min_range`：**没改**

```yaml
laser_max_range: 100.0   # 官方值
laser_min_range: -1.0    # 官方值
```

这两个是"自动"的意思：AMCL 会直接读 `/scan` 消息头里的真实量程。
URDF 里雷达量程是 `0.10~7.0m`，AMCL 自动感知，无需在参数里重复填写。

### ⑤ `map_server.yaml_filename`：留空

由 `navigation.launch.py` 的 `map` 参数注入（默认指向 `assets/maps/vi_car.yaml`）。
避免绝对路径写死在 yaml 里，方便换地图。

### ⑥ 速度参数：**没改，先跑通**

`max_vel_x: 0.26`（m/s）是 TurtleBot3 的值，本车可以先用。

**调参顺序建议**：先确认"能导航"，再调 footprint 让它贴墙准，最后调速度手感。
一开始调速度的话，出问题时无法区分是速度不合适还是 footprint 配错了。

---

## 5. 怎么跑

### 构建

URDF 改动需要重新构建才能让 `vi_car_description` 包生效：

```bash
scripts/build.sh vi_car_description
source install/setup.bash
```

### 启动

```bash
# 完整（推荐先关 Gazebo 界面省内存，你机器 14G）
ros2 launch launch/navigation.launch.py gui:=false

# 完整 + Gazebo 界面
ros2 launch launch/navigation.launch.py

# 不开 RViz（无头调试）
ros2 launch launch/navigation.launch.py gui:=false use_rviz:=false
```

### RViz 里做两件事

启动后 Nav2 不会自动知道小车在哪，必须手动告知初始位置：

1. **工具栏 → `2D Pose Estimate`** → 在地图上点击小车的**真实位置**，
   按住左键拖动设置**朝向**（车头方向）→ 松开。
   发布一个 `geometry_msgs/PoseWithCovarianceStamped`，AMCL 收到后完成初始化定位。

2. **工具栏 → `Nav2 Goal`** → 在地图上点击**目标位置**，拖动设置最终朝向 → 松开。
   发布一个目标点，Nav2 开始规划并驱动小车前往。

### 常用 launch 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `gui` | `true` | `false` = 不启动 Gazebo 图形界面 |
| `use_rviz` | `true` | `false` = 不启动 RViz |
| `map` | `assets/maps/vi_car.yaml` | 换地图 |
| `params_file` | `config/nav2/vi_car_nav2_params.yaml` | 换参数文件 |
| `autostart` | `true` | `false` = 不自动激活节点（分阶段调试用） |
| `use_composition` | `False` | `true` = 节点合并进同一容器，省内存但难排查 |

---

## 6. 验证是否接通了

启动后在另一个终端依次检查：

```bash
source install/setup.bash

# ① 地图话题：map_server 应持续发布 /map
ros2 topic echo /map --field info.resolution --once

# ② 雷达话题（Gazebo 雷达插件）
ros2 topic hz /scan

# ③ 里程计（Gazebo 差速插件）
ros2 topic echo /odom --once

# ④ TF 树：这两条是 Nav2 的生命线，必须能连通
ros2 run tf2_ros tf2_echo map base_footprint --once
ros2 run tf2_ros tf2_echo odom base_footprint --once

# ⑤ Nav2 输出：让小车动起来时应有数据
ros2 topic echo /cmd_vel --once
```

**关键：④ 的两条 TF 必须都通。**

- `map → base_footprint` 走 AMCL（需要先设初始位姿才通）
- `odom → base_footprint` 由 Gazebo 差速插件发布（`publish_odom_tf=true`）

---

## 7. 常见问题排查

| 现象 | 原因 | 处理 |
|---|---|---|
| `Could not transform from odom to base_footprint` | `robot_base_frame` 与 URDF 不一致 | 确认 4 处都是 `base_footprint` |
| AMCL 一直报 `Initial pose unknown` / 没有粒子云 | 还没在 RViz 设初始位姿 | 做 5.节第 1 步 |
| 小车原地转圈或来回抖 | footprint 太大或 `acc_lim` 太激进 | 先调 footprint，再调 `acc_lim_x` |
| 小车撞墙后 Nav2 没反应 | footprint 比真实车身小 | 放大 `footprint` |
| 规划器说找不到路径 | 起始位姿离墙太近，或膨胀半径过大 | 手动挪起点；或调 `inflation_radius` |
| Gazebo 里雷达全是 `inf` | world 是空场景，没有障碍物 | 用 `simulation.world` |
| 内存吃紧 / 卡 | 本机 14G，Gazebo + Nav2 + RViz 全开压力大 | `gui:=false`，先无头验证 |

---

## 8. 关于 Nav2 本身

### 8.1 节点构成

```
localization_launch.py
  ├─ map_server                    加载 pgm/yaml → /map
  ├─ amcl                          粒子滤波定位 → TF map→odom
  └─ lifecycle_manager_localization  统一激活上面 2 个

navigation_launch.py
  ├─ controller_server             局部避障 + 跟踪路径 → /cmd_vel_nav
  ├─ planner_server                全局规划 → /plan
  ├─ bt_navigator                  行为树调度
  ├─ behavior_server               脱困动作（旋转/后退/等待）
  ├─ smoother_server               路径平滑
  ├─ waypoint_follower             多点巡航
  ├─ velocity_smoother             速度平滑 → /cmd_vel
  └─ lifecycle_manager_navigation  统一激活上面 7 个
```

Nav2 节点都是**生命周期节点**（未激活 → 激活），必须由 `lifecycle_manager`
统一下电才能工作。所以 `autostart` 参数很关键。

`navigation.launch.py` 里 `bringup_launch.py` 已经 include 了上面两个官方 launch，
所以**不要**再单独启动 `map_server` / `amcl`，否则同名节点会抢话题。

### 8.2 可替换的算法插件（改一个字符串即可）

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["FollowPath"]   # ← 换成别的名字就换算法
```

本机 `/opt/ros/humble/lib/` 下已装的插件：

| 层 | 插件 | 库文件 | 特点 |
|---|---|---|---|
| 控制 | DWB | `libdwb_core.so` | 采样打分制，能原地转，默认 |
| 控制 | RegulatedPurePursuit | `libnav2_regulated_pure_pursuit_controller.so` | 纯路径跟踪，窄道更稳 |
| 控制 | MPPI | `libmppi_controller.so` | 模型预测，高速平滑，算力需求高 |
| 规划 | NavFn | `libnav2_navfn_planner.so` | Dijkstra 栅格，默认 |
| 规划 | SmacPlanner2D | `libnav2_smac_planner_2d.so` | 更平滑 |
| 规划 | SmacPlannerLattice | `libnav2_smac_planner_lattice.so` | 支持阿克曼等运动学约束 |

**这些算法不区分底盘类型**——Nav2 只输出 `(v, ω)`，
所以麦轮、阿克曼、履带车都能用（阿克曼需要配 `SmacPlannerLattice`，
因为它不能原地掉头）。

### 8.3 `map → odom` 的其他提供者

AMCL 不是唯一选择。以下方案互斥，选其一：

| 方案 | `map → odom` 由谁发 | 特点 |
|---|---|---|
| **A. AMCL**（本项目采用） | `nav2_amcl` | 只需 pgm/yaml，与建图解耦，最易上手 |
| B. Cartographer 纯定位 | `cartographer_node`（`pure_localization` 模式） | 扫描匹配精度更高，需加载 `.pbstream` |
| C. slam_toolbox 定位器 | `slam_toolbox_localization` | 本机已装，有 5 种优化后端（G2O/GTSAM/Ceres/...） |
| D. `robot_localization` EKF | EKF 节点 | 融合 IMU + 轮速 + 可选 GPS |

本机已装的 2D SLAM：`cartographer_ros`、`slam_toolbox`。
**Cartographer 本身也支持 3D**（`MAP_BUILDER.use_trajectory_builder_3d = true`），
本项目用的是 2D 模式（`use_trajectory_builder_2d = true`）。

---

## 9. 学习建议

1. **先跑通再调参**：官方值全都能跑，不要一上来就改一堆参数，否则无法定位问题。
2. **改一个参数 → 观察一个现象**。这是学 Nav2 最有效的方式。
3. **重点理解 `footprint` 和 `inflation_radius`**：这两个决定"车怎么避障"，
   是所有导航问题里最常调的。
4. **换地图场景测试**：`world:=<你的.world>` 换个房间，
   观察 local_costmap 的滚动窗口如何跟随小车。
5. **试换控制器**：`controller_plugins` 在 DWB / RPP / MPPI 之间切换，
   对比同样场景下路径跟踪的差异。

官方文档：<https://docs.nav2.org>（选 Humble 版本）
参数含义：每个参数在 `nav2_params.yaml` 官方文件里都有行内注释说明。