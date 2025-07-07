include "map_builder.lua"
include "trajectory_builder.lua"

options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    -- tracking_frame = "base_link",    -- for real robot
    tracking_frame = "base_footprint",  -- for turtlebot3 sim 
    published_frame = "odom",
    odom_frame = "odom",
    provide_odom_frame = false, 

    publish_frame_projected_to_2d = false,


    use_odometry = true,                                  -- 是否使用里程计,如果使用要求一定要有odom的tf
    use_nav_sat = false,                                  -- 是否使用gps
    use_landmarks = false,                                -- 是否使用landmark
    num_laser_scans = 1,                               -- 激光雷达数量
    num_multi_echo_laser_scans = 0,                      -- 多回波激光雷达数量
    num_subdivisions_per_laser_scan = 1,                  -- 每个激光雷达的子划分数量
    num_point_clouds = 0,                                -- 点云数量
    
    lookup_transform_timeout_sec = 0.2,                  -- 查找tf变换的超时时间
    submap_publish_period_sec = 0.3,                     -- 子图发布周期
    pose_publish_period_sec = 5e-3,                       -- 位姿发布周期
    trajectory_publish_period_sec = 30e-3,                -- 轨迹发布周期
    rangefinder_sampling_ratio = 1.,                      -- 激光雷达采样率
    odometry_sampling_ratio = 1.,                         -- 里程计采样率
    fixed_frame_pose_sampling_ratio = 1.,                 -- 固定帧位姿采样率
    imu_sampling_ratio = 1.,                              -- IMU采样率
    landmarks_sampling_ratio = 1.,                        -- 地标采样率
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
-- TRAJECTORY_BUILDER_2D.min_range = 0.3                --laser min_range
-- TRAJECTORY_BUILDER_2D.max_range = 6.                 --laser max_range    
--TRAJECTORY_BUILDER_2D.min_z = 0.2
--TRAJECTORY_BUILDER_2D.max_z = 1.4
--TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02

--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.5
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200.
--TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_range = 50.

--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.9
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.min_num_points = 100
--TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_range = 50.

TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.5
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
--TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 12

--TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.1
--TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = 0.004
--TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 1.

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1

POSE_GRAPH.optimize_every_n_nodes = 50.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.
POSE_GRAPH.constraint_builder.min_score = 0.48
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options