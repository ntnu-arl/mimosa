// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/lidar/geometric_config.hpp"

namespace mimosa
{
namespace lidar
{
void declare_config(RegistrationConfig & config)
{
  using namespace config;
  name("Registration Config");
  field(config.source_voxel_grid_filter_leaf_size, "source_voxel_grid_filter_leaf_size", "m");
  field(config.source_voxel_grid_min_dist_in_voxel, "source_voxel_grid_min_dist_in_voxel", "m");
  field(config.target_ivox_map_leaf_size, "target_ivox_map_leaf_size", "m");
  field(config.target_ivox_map_min_dist_in_voxel, "target_ivox_map_min_dist_in_voxel", "m");
  field(config.num_corres_points, "num_corres_points", "num");
  field(config.max_corres_distance, "max_corres_distance", "m");
  field(config.plane_validity_distance, "plane_validity_distance", "m");
  field(config.lidar_point_noise_std_dev, "lidar_point_noise_std_dev", "m");
  field(config.use_huber, "use_huber", "bool");
  field(config.huber_threshold, "huber_threshold", "amount in standard deviations");
  field(config.reg_4_dof, "reg_4_dof", "bool");
  field(config.project_on_degneneracy, "project_on_degneneracy", "bool");
  field(config.degen_thresh_rot, "degen_thresh_rot", "num_virtual_features");
  field(config.degen_thresh_trans, "degen_thresh_trans", "num_virtual_features");

  check(config.source_voxel_grid_filter_leaf_size, GE, 0.01, "source_voxel_grid_filter_leaf_size");
  check(
    config.source_voxel_grid_min_dist_in_voxel, GE, 0.01, "source_voxel_grid_min_dist_in_voxel");
  checkCondition(
    config.source_voxel_grid_min_dist_in_voxel < config.source_voxel_grid_filter_leaf_size,
    "source_voxel_grid_min_dist_in_voxel should be smaller than "
    "source_voxel_grid_filter_leaf_size");
  check(config.target_ivox_map_leaf_size, GE, 0.01, "target_ivox_map_leaf_size");
  check(config.target_ivox_map_min_dist_in_voxel, GE, 0.01, "target_ivox_map_min_dist_in_voxel");
  checkCondition(
    config.target_ivox_map_min_dist_in_voxel < config.target_ivox_map_leaf_size,
    "target_ivox_map_min_dist_in_voxel should be smaller than target_ivox_map_leaf_size");
  check(config.num_corres_points, GE, 3, "num_corres_points");
  check(config.max_corres_distance, GT, 0.0, "max_corres_distance");
  check(config.plane_validity_distance, GT, 0.0, "plane_validity_distance");
  check(config.lidar_point_noise_std_dev, GT, 0.0, "lidar_point_noise_std_dev");
  check(config.huber_threshold, GE, 0.0, "huber_threshold");
}

void declare_config(GeometricConfig & config)
{
  using namespace config;
  name("Lidar Geometric Config");

  field(config.logs_directory, "logs_directory", "directory_path");
  field(config.world_frame, "world_frame", "str");
  field(config.body_frame, "body_frame", "str");

  {
    NameSpace ns("lidar");
    field(config.T_B_L, "T_B_S", "gtsam::Pose3");
    field(config.sensor_frame, "sensor_frame", "str");
    {
      NameSpace ns("geometric");
      field(config.enabled, "enabled", "bool");
      field(config.log_level, "log_level", "trace|debug|info|warn|error|critical");
      field(
        config.point_skip_divisor, "point_skip_divisor", "divisor for downsampling the pointcloud");
      field(
        config.ring_skip_divisor, "ring_skip_divisor", "divisor for downsampling the pointcloud");
      field(config.map_keyframe_trans_thresh, "map_keyframe_trans_thresh", "m");
      field(config.map_keyframe_rot_thresh_deg, "map_keyframe_rot_thresh_deg", "deg");
      field(config.initial_clouds_to_force_map_update, "initial_clouds_to_force_map_update", "num");
      field(config.lru_horizon, "lru_horizon", "num");
      field(config.neighbor_voxel_mode, "neighbor_voxel_mode", "num");

      field(config.scan_to_map, "scan_to_map");
    }
  }

  check(config.point_skip_divisor, GE, 1, "point_skip_divisor");
  check(config.ring_skip_divisor, GE, 1, "ring_skip_divisor");
  std::vector<size_t> neighbor_voxel_mode_values = {1, 7, 19, 27};
  check(
    std::find(
      neighbor_voxel_mode_values.begin(), neighbor_voxel_mode_values.end(),
      config.neighbor_voxel_mode) != neighbor_voxel_mode_values.end(),
    EQ, true, "neighbor_voxel_mode");
}

}  // namespace lidar
}  // namespace mimosa
