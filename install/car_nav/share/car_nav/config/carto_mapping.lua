-- 引入 Cartographer 默认配置片段
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "laser_frame",
  published_frame = "base_footprint", 
  odom_frame = "odom_frame",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = true,  -- 启用里程计数据，提高定位稳定性
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
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

-- 调整激光雷达参数，适应13Hz频率
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 增强扫描匹配稳定性
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = 0.2
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- 优化Ceres求解器参数
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 50.0  -- 增加平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 100.0   -- 增加旋转权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30

-- 增加累积扫描数据，提高匹配质量
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2

-- 降低后端优化频率，提高实时性
POSE_GRAPH.optimize_every_n_nodes = 120
POSE_GRAPH.constraint_builder.min_score = 0.50  -- 稍微降低约束阈值
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55

return options
