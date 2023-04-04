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


include "map_builder.lua"
include "trajectory_builder.lua"



options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  -- publish_tracked_pose = true,
  -- use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 10,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 10e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
publish_tracked_pose = true
use_pose_extrapolator = true
-- TRAJECTORY_BUILDER.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }

-- TRAJECTORY_BUILDER.pure_localization_trimmer = {
--   max_submaps_to_keep = 3,
-- }


TRAJECTORY_BUILDER.pure_localization = true
POSE_GRAPH.optimize_every_n_nodes = 20

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1e2
-- MAP_BUILDER.num_background_threads = 6
POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001
-- POSE_GRAPH.optimize_every_n_nodes = 1
-- POSE_GRAPH.optimize_every_n_nodes = 10
POSE_GRAPH.optimization_problem.huber_scale = 1e3 --큰 값을 설정하면 outlier에 대한 영향을 줄임
POSE_GRAPH.global_sampling_ratio = 0.0001

return options
