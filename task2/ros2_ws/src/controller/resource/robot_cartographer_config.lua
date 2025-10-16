-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- /* Author: Darby Lim */

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.1,
  trajectory_publish_period_sec = 0.1,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.5,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 7.5
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.05)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

POSE_GRAPH.optimize_every_n_nodes = 30

return options

-- Описание TRAJECTORY_BUILDER_2D
-- TRAJECTORY_BUILDER_2D = {
--   use_imu_data = true,
--   min_range = 0.,
--   max_range = 30.,
--   min_z = -0.8,
--   max_z = 2.,
--   missing_data_ray_length = 5.,
--   num_accumulated_range_data = 1,
--   voxel_filter_size = 0.025,

--   adaptive_voxel_filter = {
--     max_length = 0.5,
--     min_num_points = 200,
--     max_range = 50.,
--   },

--   loop_closure_adaptive_voxel_filter = {
--     max_length = 0.9,
--     min_num_points = 100,
--     max_range = 50.,
--   },

--   use_online_correlative_scan_matching = false,
--   real_time_correlative_scan_matcher = {
--     linear_search_window = 0.1,
--     angular_search_window = math.rad(20.),
--     translation_delta_cost_weight = 1e-1,
--     rotation_delta_cost_weight = 1e-1,
--   },

--   ceres_scan_matcher = {
--     occupied_space_weight = 1.,
--     translation_weight = 10.,
--     rotation_weight = 40.,
--     ceres_solver_options = {
--       use_nonmonotonic_steps = false,
--       max_num_iterations = 20,
--       num_threads = 1,
--     },
--   },

--   motion_filter = {
--     max_time_seconds = 5.,
--     max_distance_meters = 0.2,
--     max_angle_radians = math.rad(1.),
--   },

--   imu_gravity_time_constant = 10.,

--   submaps = {
--     resolution = 0.05,
--     num_range_data = 90,
--     range_data_inserter = {
--       insert_free_space = true,
--       hit_probability = 0.55,
--       miss_probability = 0.49,
--     },
--   },
-- }