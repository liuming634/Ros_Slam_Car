# 对接手册：Cartographer SLAM + Nav2 导航 × vi_car 小车

本文回答一个问题：**Cartographer 和 Nav2 都是现成的标准件（相当于"pip install 完直接 import 的包"），
它们各自要求什么输入？本项目在哪一个文件的哪一处把输入递了过去？为什么这么接？**

这就是"对接层"的完整说明书。日常操作命令（怎么启动、怎么建图、怎么发目标）不在这里：

- 建图操作与排障 → [docs/slam.md](slam.md)
- 导航操作与 Nav2 参数逐项详解 → [docs/navigation.md](navigation.md)

---

## 0. 总图：三个标准件要什么，我们递了什么

| 标准件 | 它要求的输入 | 本项目由谁提供 | 对接文件 |
|---|---|---|---|
| **Cartographer**（建图） | `/scan`、`/imu/data`、能查到的静态 TF（`base_link→laser_link/imu_link`）、lua 配置 | Gazebo 雷达/IMU 插件、robot_state_publisher、`cartographer.launch.py` | `config/cartographer/vi_car_2d.lua` |
| **map_server + AMCL**（定位） | 地图 `yaml+pgm`、`/scan`、TF `odom→base_footprint`、初始位姿 `/initialpose` | Cartographer 产物、Gazebo 差速插件、手动发布 | `launch/navigation.launch.py` |
| **Nav2 七节点**（规划控制） | `/map`、`/scan`、完整 TF 链 `map→odom→base_footprint`、参数文件 | map_server、AMCL、robot_state_publisher | `config/nav2/vi_car_nav2_params.yaml` |
| **Gazebo 底盘**（执行） | `/cmd_vel`（Twist） | Nav2 velocity_smoother 输出 | URDF 里的差速插件 |

三条对接纪律（违反任何一条，链路必挂，症状还很迷惑）：

1. **TF 每条边只能有一个写入者**。本项目最典型：`odom` 段在建图时归 Cartographer、
   导航时归差速插件，靠两份 URDF 的 `publish_odom_tf` 开关互斥（见 §2.4）。
2. **话题名和消息类型要真实接上**。插件默认发 `~/out` 这类私有名，靠 `<remapping>`
   接到约定名；launch 里 `imu→imu/data` 同理。
3. **所有节点对"现在几点"的认知必须一致**。仿真里全靠 Gazebo 的 `/clock`，
   任何一个节点 `use_sim_time` 没开，就会出现"消息时间戳对不上 TF"的幽灵故障（见 §5）。

---

## 1. 对接只动四类文件

```
src/robot/vi_car_description/urdf/   ① 车模 + Gazebo 插件（传感器→话题、差速→/cmd_vel）
launch/                              ② 装配：把标准件和 ③④ 拼在一起、注入参数
config/cartographer/vi_car_2d.lua    ③ Cartographer 参数
config/nav2/vi_car_nav2_params.yaml  ③ Nav2 参数
assets/maps/                         ④ 地图产物（建图的输出 = 定位的输入）
```

对接时的"万用表"就两条命令：

```bash
ros2 topic list                      # 话题接上了没有、名字对不对
ros2 run tf2_tools view_frames       # TF 树长什么样、谁在写哪条边
```

---

## 2. 车模层对接（URDF 里的插件）

URDF 做两件事：给 robot_state_publisher 提供静态 TF 骨架；给 Gazebo 提供
传感器插件和差速插件。**SLAM 和 Nav2 感知到的"车"完全由这一层决定。**

### 2.1 坐标系骨架

```
base_footprint（地面投影原点）──► base_link（车体原点，在轮轴高度）
                                      ├──► laser_link   （雷达）
                                      └──► imu_link     （IMU）
```

为什么分 `base_footprint` 和 `base_link` 两个：底盘中心在轮轴高度（z≠0），
而导航/AMCL/代价地图需要的是**贴地的投影原点**。本项目五个地方都统一用
`base_footprint`，见 §4.1 一致性清单。

### 2.2 雷达插件（`vi_car.urdf` 的 `<sensor name="vi_car_lidar" type="ray">`）

```xml
<update_rate>10</update_rate>            <!-- 10Hz 扫描 -->
<samples>720</samples>                    <!-- 360° 均分 720 点，0.5°/点 -->
<min>0.10</min> <max>7.0</max>            <!-- 量程：下游所有参数都要对齐它 -->
<plugin filename="libgazebo_ros_ray_sensor.so">
  <remapping>~/out:=scan</remapping>      <!-- ★ 对接点：私有名 → /scan -->
  <output_type>sensor_msgs/LaserScan</output_type>
  <frame_name>laser_link</frame_name>     <!-- ★ 对接点：消息挂在哪个 frame -->
</plugin>
```

下游三个地方消费这个量程，**换雷达/改量程时四处要同步**：

| 消费者 | 参数 | 本项目值 |
|---|---|---|
| Cartographer lua | `TRAJECTORY_BUILDER_2D.min_range / max_range` | 0.15 / 12.0（放宽，实际由消息截断） |
| Nav2 costmap | `obstacle_max_range`（必须 ≤ 7.0，否则幽灵障碍） | 5.0 |
| AMCL | `laser_max_range: 100.0` = 自动读消息头 | 不用改 |

### 2.3 IMU 插件（`<sensor name="vi_car_imu" type="imu">`）

```xml
<update_rate>100</update_rate>
<plugin filename="libgazebo_ros_imu_sensor.so">
  <remapping>~/out:=imu/data</remapping>          <!-- ★ 对接点 -->
  <!-- ★★ 必踩坑：false 才输出 ENU 世界系姿态（重力 -Z 对齐），
       默认 true 输出的是相对初始姿态的姿态，Cartographer 2D 会直接拒收 -->
  <initial_orientation_as_reference>false</initial_orientation_as_reference>
  <frame_name>imu_link</frame_name>
</plugin>
```

launch 里再补一刀 remap（`cartographer.launch.py`）：`('imu', 'imu/data')`，
因为 Cartographer 的 lua 侧固定订阅 `imu` 这个名字。

### 2.4 差速插件——两份 URDF 的唯一差异（本项目核心设计）

```xml
<plugin filename="libgazebo_ros_diff_drive.so">
  <num_wheel_pairs>2</num_wheel_pairs>       <!-- 四轮差速必须声明，默认 1 会报 joints 不一致 -->
  <wheel_separation>0.125</wheel_separation> <!-- 与 Nav2 footprint、实车实物三者手工同步 -->
  <wheel_diameter>0.05</wheel_diameter>
  <max_wheel_torque>10.0</max_wheel_torque>          <!-- 原 5.0 起步发不出力，车"按了没反应" -->
  <max_wheel_acceleration>50.0</max_wheel_acceleration>
  <publish_odom>true</publish_odom>          <!-- /odom 话题始终发布（AMCL/vel_smoother 用） -->
  <publish_odom_tf>false</publish_odom_tf>   <!-- ★ 唯一开关：见下表 -->
  <robot_base_frame>base_footprint</robot_base_frame>
</plugin>
```

| | `vi_car.urdf`（建图） | `vi_car_nav.urdf`（导航） |
|---|---|---|
| `publish_odom_tf` | **false** | **true** |
| `odom→base_footprint` 这条 TF 谁写 | 没人写（Cartographer 直接发 `odom→base_link`） | 差速插件 |
| 为什么 | lua 里 `provide_odom_frame=true`，Cartographer 自己造 odom 段。若插件也发，**两个写入者打架**，TF 抖动，建图立刻废 | 导航模式没有 Cartographer，没人发这条 TF，AMCL 会直接报 `Could not transform` |

> 实物车用 ros2_control 时对应物是 `diff_drive_controller`：它的
> `use_odom_tf` 同样要遵守"这条边只有一个人写"的纪律；且 Humble 里它
> 默认订阅 `~/cmd_vel_unstamped`，要把 `/cmd_vel` remap 过去。

---

## 3. Cartographer 对接（`config/cartographer/vi_car_2d.lua`）

### 3.1 它的输入接口（lua 顶部四个 frame 决定一切）

```lua
tracking_frame  = "imu_link",   -- 所有传感器数据最终被变换到这个帧再进算法。
                                -- 选 imu_link 是官方推荐（IMU 频率最高、不做外参插值）；
                                -- 条件：robot_state_publisher 必须能提供 base_link→imu_link
published_frame = "base_link",  -- 它对外发布的 TF 挂到这个帧：输出 odom→base_link
provide_odom_frame = true,      -- 由它造 odom 段（对应 §2.4 插件 publish_odom_tf=false）
use_odometry    = false,        -- 没有轮式里程计可喂；实车接上编码器 odom 后改 true
num_laser_scans = 1,            -- 消费 1 路 /scan
```

**对接自查法**：lua 改完，问自己三个问题——
① `tracking_frame` 能不能通过静态 TF 从 URDF 到达？
② `published_frame + provide_odom_frame` 组合发出的 TF 段，有没有和别的节点冲突？
③ `use_*` 开关和实际存在的话题数量是否一致？（`num_laser_scans=1` 但没有 `/scan`
就会无限刷 `Queue waiting for data`）

### 3.2 关键算法开关与"为什么"

| 参数 | 值 | 为什么 |
|---|---|---|
| `MAP_BUILDER.use_trajectory_builder_2d` | true | 2D 建图 |
| `TRAJECTORY_BUILDER_2D.use_imu_data` | true | 与 IMU 插件的 ENU 约定（§2.3）配套 |
| `use_online_correlative_scan_matching` | true | 没有轮式里程计时的兜底：用相关匹配抗快速旋转/里程漂移 |
| `min_range/max_range` | 0.15/12.0 | 覆盖雷达真实量程 0.10~7.0 |
| `POSE_GRAPH.optimize_every_n_nodes` | 35 | 回环检测频率；房间小、轨迹简单，默认值够用 |

### 3.3 产出与保存链（易断的三处）

```
cartographer_node ──► cartographer_occupancy_grid_node ──► /map（RViz 实时预览）
        │                    （-resolution 0.05，与 Nav2 costmap resolution 一致）
        └─ save_map:=true 时带 -save_state_filename=assets/maps/vi_car.pbstream
                                │
                                ▼  ★ 只在进程优雅退出时写盘
                    Ctrl+C（等 "failed to terminate" 警告别出现）
                                │
                                ▼
              ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
                     -pbstream_filename assets/maps/vi_car.pbstream
              ★ 输出固定落在【当前工作目录】的 map.pgm / map.yaml
                                │
                                ▼
              mv 到 assets/maps/ + sed 改 yaml 的 image: 字段指向 vi_car.pgm
```

三个已验证的坑：

1. **SIGKILL / 直接关终端 = pbstream 不落盘**（旧文件原样留着，极易误用）。
   转换前必须 `ls -l` 核对 `vi_car.pbstream` 的时间戳和大小。
2. **转换工具不认路径**，永远输出到 CWD，要手动搬 + 改 yaml 的 `image:` 字段。
3. yaml 的 `image:` 是相对 yaml 自身的路径，`map_server` 按此解析——
   搬完不改就会报 "failed to open image file"。

---

## 4. Nav2 对接（`config/nav2/vi_car_nav2_params.yaml`）

Nav2 要的东西在 §0 表里列全了，参数逐项详解在
[docs/navigation.md](navigation.md) 第 4 节，这里只列**对接矩阵**——
每个输入从哪根线接进来。

### 4.1 `base_footprint` 一致性清单（五处，改车必查）

```
① URDF 差速插件  <robot_base_frame>        ← TF odom→base_footprint 挂哪
② amcl.base_frame_id
③ bt_navigator.robot_base_frame
④ local_costmap.robot_base_frame
⑤ global_costmap.robot_base_frame
```

任何一处写成 `base_link`，症状都是 AMCL/costmap 持续刷
`Could not transform from odom to base_footprint`。

### 4.2 `/cmd_vel` 供电链（谁发到谁）

```
controller_server ──remap──► /cmd_vel_nav ──► velocity_smoother ──► /cmd_vel ──► Gazebo 差速插件
      （DWB 采样打分）              （加速度限幅、平滑）           （换算轮速）
```

Humble 官方 bringup 已把这条链的 remap 配好，**不要自己再串一个速度节点进去**。
实车换 ros2_control 时把最后一个箭头指向 `diff_drive_controller`（注意
`cmd_vel_unstamped` remap）。

### 4.3 launch 装配纪律（`launch/navigation.launch.py`）

- 只 include 官方 `bringup_launch.py`（它内部已含 localization + navigation
  两个子 launch），**绝不**再单独起 map_server/amcl——同名节点会抢话题；
- `slam:='False'` 显式写死，防止上游默认值变化时 slam_toolbox 被拉起来抢 `/map`；
- `map`/`params_file` 两个 LaunchConfiguration 注入官方节点，路径不写死在 yaml 里。

### 4.4 程序化导航 API（nav2_simple_commander，Humble 自带）

Nav2 官方 Python 高层封装，随 Nav2 一起安装，把"初始位姿 → 等激活 → 发目标 →
收反馈 → 判结果"包成方法调用，是**多点巡航/巡检任务**的正经写法。底层仍是
`navigate_to_pose` action（§4.2 同源），本项目实测的 FastDDS 响应超时（§6 #6）
对它同样可能发生，处理方式一致（重发 / UDP-only profile）。

**★ Humble 与网上教程（新版 API）的名字差异**，照抄新版会 AttributeError：

| 教程（Iron+） | **Humble 实际** |
|---|---|
| `NavigationResult` | `TaskResult`（UNKNOWN/SUCCEEDED/CANCELED/FAILED） |
| `isNavComplete()` | `isTaskComplete()` |
| `cancelNav()` | `cancelTask()` |
| `feedback.navigation_duration` | `feedback.navigation_time`（Duration，用 `Duration.from_msg()` 比较） |

本项目用法模板（存成 `scripts/nav_commander.py`，跑在已激活的导航链之上）：

```python
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

rclpy.init()
nav = BasicNavigator()

initial = PoseStamped()                     # ① 初始位姿（Gazebo 原点）
initial.header.frame_id = 'map'
initial.header.stamp = nav.get_clock().now().to_msg()
initial.pose.orientation.w = 1.0
nav.setInitialPose(initial)

nav.waitUntilNav2Active()                   # ② 必须在 ① 之后，否则 AMCL 在原点乱初始化

goal = PoseStamped()                        # ③ 目标点（参考 §5 选位）
goal.header.frame_id = 'map'
goal.header.stamp = nav.get_clock().now().to_msg()
goal.pose.position.x = -3.5
goal.pose.position.y = -1.5
goal.pose.orientation.w = 1.0
nav.goToPose(goal)

while not nav.isTaskComplete():             # ④ 轮询 + 超时取消
    fb = nav.getFeedback()
    if fb and Duration.from_msg(fb.navigation_time) > Duration(seconds=600.0):
        nav.cancelTask()

print('result:', nav.getResult())           # ⑤ TaskResult.SUCCEEDED 即到达
nav.lifecycleShutdown()
rclpy.shutdown()
```

```bash
# 仿真里必须带仿真时钟，否则节点走墙钟、stamp 与全系统错位：
python3 scripts/nav_commander.py --ros-args -p use_sim_time:=true
```

进阶：`goThroughPoses`/`followWaypoints`（多点）、`getPath`（只规划不执行）、
`spin`/`backup`（脱困）、`getGlobalCostmap`、`changeMap`（运行时换图）、
`lifecycleStartup/Shutdown`（配 `autostart:=false` 分阶段调试）。

---

## 5. 时间同步对接（最隐蔽的一层）

```
gzserver ──► /clock ──► 所有 use_sim_time:=true 的节点
                          （Cartographer、AMCL、Nav2 七节点、RViz）
```

- 仿真模式：`cartographer.launch.py` 的 `use_sim_time` 默认随 `use_sim:=true`
  自动置 true；`navigation.launch.py` 固定 true。
- **症状识别**：日志里出现 `Message Filter dropping ... timestamp earlier than
  buffer`、`Lookup would require extrapolation`，优先怀疑某个节点没吃 `/clock`，
  而不是去调 TF 参数。
- 实车模式：`use_sim:=false` → `use_sim_time` 自动回落 false，全系统走墙钟，
  这也是 `use_sim` 开关存在的意义之一。

---

## 6. 对接事故登记表（本项目真实踩过的坑）

| # | 坑 | 症状 | 根因 | 修复 | 一句话教训 |
|---|---|---|---|---|---|
| 1 | 空世界建不出图 | Cartographer 无限刷 `Queue waiting for data: (0, scan)` | 默认空场景无障碍，雷达全是 inf，点云被静默丢弃 | 默认加载含房间的 `simulation.world` | 传感器没有回波，SLAM 端不报错只装死 |
| 2 | TF 双源打架 | 建图漂移/TF 抖动 | 差速插件和 Cartographer 都想写 odom 段 | 两份 URDF 的 `publish_odom_tf` 互斥 | TF 每条边一个写入者 |
| 3 | LaunchConfiguration 传参报错 | `expected str, bytes or os.PathLike object, not LaunchConfiguration` | 在 launch 求值前就想用运行时参数拼路径 | `OpaqueFunction` 延迟求值 | launch 是声明式描述，不是立即执行的脚本 |
| 4 | 车不听遥控 | 遥控发了车只蠕动 | `max_wheel_torque=5.0` 对 0.6kg 车太保守 + 轮子摩擦 | torque 10 / accel 50 / mu2=0.01 | Gazebo 物理参数会伪装成"软件 bug" |
| 5 | 建完图文件是旧的 | 转出来的地图还是上次的内容 | SIGKILL/关终端导致 pbstream 没落盘 | Ctrl+C 等写完 + 转换前核对时间戳 | 优雅退出是保存流程的一部分 |
| 6 | 发目标车不动 | `Failed to send goal response (timeout)` | FastDDS 共享内存偶发故障（话题正常、仅 action 响应超时） | 重发一次；根治用 UDP-only XML profile | 中间件层故障长得像业务 bug，先看通信再看逻辑 |

---

## 7. 对接验证流程（每次大改动后跑一遍）

```bash
# ---- 静态（不启动仿真）----
bash scripts/build.sh -j6 && source install/setup.bash
ros2 pkg prefix vi_car_description          # 包能被找到

# ---- 动态（启动对应链路后，另开终端）----
ros2 topic hz /scan                         # 雷达 ~10Hz
ros2 topic hz /imu/data                     # IMU ~100Hz
ros2 topic echo /odom --once                # 差速插件里程计在跳
ros2 topic echo /map --once                 # 建图: occupancy_grid；导航: map_server（需 --qos-durability transient_local）
ros2 run tf2_ros tf2_echo map base_footprint --ros-args -p use_sim_time:=true
ros2 run tf2_ros tf2_echo odom base_footprint --ros-args -p use_sim_time:=true
ros2 topic echo /cmd_vel --once             # 导航中应有数据流；发目标后 /plan 出现
```

判定标准：**§0 表里每个"输入"都能在 `ros2 topic list` 里找到、每条 TF 边
都能在 `view_frames` 里连通**，对接就是完整的。

---

## 8. 迁移到实车：接口不变，换提供者

| 接口 | 仿真提供者 | 实车提供者 | 要动的文件 |
|---|---|---|---|
| `/scan` | Gazebo ray 插件 | YDLidar 驱动（`use_sim:=false` 已预留 `hardware_sensors.launch.py`） | 驱动参数：量程、频率 |
| `/imu/data` | Gazebo IMU 插件 | 实体 IMU 驱动（保持 ENU 约定！） | 驱动配置 |
| `/odom` + TF | Gazebo 差速插件 | 编码器 → `diff_drive_controller`（或自写节点） | URDF/控制器配置 |
| `/cmd_vel` 消费 | 差速插件 | 电机驱动（速度闭环，勿开环） | 底盘固件 |
| 时钟 | Gazebo `/clock` | 墙钟（`use_sim_time:=false` 自动生效） | 无 |
| 速度上限 | 0.26 m/s（TB3 遗留值） | 按实车电机实测改 | `vi_car_nav2_params.yaml` |

不变的东西：lua 的 frame 结构、`base_footprint` 五处一致性、
`/cmd_vel` 契约、地图产物格式、两条链路的互斥关系。
**这就是对接做干净的回报——换底盘只换最后一排。**
