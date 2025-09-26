-- Cartographer 纯定位模式配置
include "map_builder.lua"  -- 包含地图构建器配置
include "trajectory_builder.lua"  -- 包含轨迹构建器配置

options = {
  map_builder = MAP_BUILDER,  -- 地图构建器对象
  trajectory_builder = TRAJECTORY_BUILDER,  -- 轨迹构建器对象
  map_frame = "map",  -- 地图坐标系名称
  tracking_frame = "laser_frame",  -- 跟踪坐标系名称
  published_frame = "base_footprint",  -- 发布的机器人坐标系名称
  odom_frame = "odom_frame",  -- 里程计坐标系名称
  provide_odom_frame = true,  -- 是否提供里程计坐标系
  publish_frame_projected_to_2d = false,  -- 是否将坐标系投影到2D平面
  use_odometry = true,  -- 使用里程计数据
  use_nav_sat = false,  -- 不使用导航卫星数据
  use_landmarks = false,  -- 不使用地标数据
  num_laser_scans = 1,  -- 激光扫描器数量
  num_multi_echo_laser_scans = 0,  -- 多回波激光扫描器数量
  num_subdivisions_per_laser_scan = 1,  -- 每次激光扫描的子扫描数量
  num_point_clouds = 0,  -- 点云数据数量
  lookup_transform_timeout_sec = 0.2,  -- 查找坐标变换的超时时间（秒）
  submap_publish_period_sec = 0.3,  -- 子地图发布周期（秒）
  pose_publish_period_sec = 5e-3,  -- 位姿发布周期（秒）
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期（秒）
  rangefinder_sampling_ratio = 1.,  -- 距离传感器采样比例
  odometry_sampling_ratio = 1.,  -- 里程计采样比例
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定坐标系位姿采样比例
  imu_sampling_ratio = 1.,  -- IMU采样比例
  landmarks_sampling_ratio = 1.,  -- 地标采样比例
}

-- 启用纯定位模式 - 正确的语法  
MAP_BUILDER.use_trajectory_builder_2d = true  -- 使用2D轨迹构建器
-- 注意：纯定位模式通过加载地图文件和配置参数自动启用，不需要pure_localization参数

-- 定位模式的扫描匹配参数（更保守）
TRAJECTORY_BUILDER_2D.min_range = 0.3  -- 激光最小范围（米）
TRAJECTORY_BUILDER_2D.max_range = 12.0  -- 激光最大范围（米）
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- 缺失数据的射线长度（米）
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 不使用IMU数据

-- 定位模式下更严格的匹配参数
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- 线性搜索窗口（米）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = 0.15  -- 角度搜索窗口（弧度）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.0  -- 平移代价权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1  -- 旋转代价权重

-- 提高定位精度
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 70.0  -- 平移权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 200.0  -- 旋转权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20  -- Ceres求解器最大迭代次数

-- 定位模式：不累积太多数据
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 累积的激光数据数量

-- 简化后端优化（定位模式）
POSE_GRAPH.optimize_every_n_nodes = 90  -- 每90个节点优化一次

return options  -- 返回配置选项
