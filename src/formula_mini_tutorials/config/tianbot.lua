include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "tianracer/base_link",  -- 替换成机器人实际的基坐标系
  published_frame = "tianracer/odom",      -- 替换成机器人实际的里程计坐标系
  odom_frame = "tianracer/odom",          -- 和上面保持一致

  provide_odom_frame = false,      -- 如果你的底盘驱动已经发布了 odom->base_link，这里必须是 false
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = true,             -- 建议开启里程计
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,             -- 单线雷达数量
  -- 补充多回波激光扫描数量（不用就设为 0）
  num_multi_echo_laser_scans = 0.0,
    -- 可选补充：如果后续还报其他参数缺失，可一并补充以下常用参数
  

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

-- 2D 参数微调
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 8.0      -- 根据你的雷达最大距离修改
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false -- 如果没有高精度IMU，强烈建议设为 false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

return options