-- ============================================================
-- ROS SLAM 小车 - Cartographer 2D 建图配置（激光雷达为主）
-- 数据链路：
--   lidar_node(/scan) ─┐
--   imu_node(/imu/data)├─> cartographer_node -> /map + /tf(map->odom->base_link)
--   (camera_node 保留接口 /camera/image_raw，Cartographer 不消费)
--
-- 坐标系：
--   map(全局, cartographer) -> odom(cartographer 发布) -> base_link
--     -> laser_link / camera_link / imu_link (urdf/vi_car.urdf 静态 TF)
--
-- 启动方式：ros2 launch launch/cartographer.launch.py
-- ============================================================

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- ---------- TF 坐标系 ----------
  map_frame = "map",              -- 全局地图坐标系
  -- tracking_frame 用 imu_link：Cartographer 的 IMU 数据不做坐标系变换，
  -- 必须直接落在 tracking_frame 上（官方 mir-100 同款用法）。
  -- 若不用 IMU（use_imu_data=false），可改回 "base_link" 或 "laser_link"。
  tracking_frame = "imu_link",    -- 被估计的跟踪坐标系（IMU 帧）
  published_frame = "base_link",  -- cartographer 发布 odom->base_link
  odom_frame = "odom",
  provide_odom_frame = true,      -- 纯雷达 SLAM：由 cartographer 提供 odom
  publish_frame_projected_to_2d = true,

  -- ---------- 传感器 ----------
  -- 默认订阅话题：/scan（激光）、/imu（IMU，launch 里 remap 到 /imu/data）
  use_odometry = false,           -- 暂无轮式里程计；接入后改 true 并发布 /odom
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,            -- 1 路 2D 激光雷达
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- ---------- 发布频率 / 采样 ----------
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ---------- 2D 激光雷达参数 ----------
-- RPLIDAR A1 等：量程约 0.15m~12m；换雷达时按实际量程调整下面两项
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0
-- 有 IMU 接入时置 true（Cartographer 用 IMU 修正旋转/重力对齐）；
-- 无 IMU 时置 false 也照样能建图
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35

-- ---------- 回环 / 全局优化 ----------
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
