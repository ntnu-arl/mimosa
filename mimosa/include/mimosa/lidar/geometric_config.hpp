// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "mimosa/lidar/utils.hpp"
#include "mimosa/utils.hpp"

namespace mimosa
{
namespace lidar
{

struct RegistrationConfig
{
  float source_voxel_grid_filter_leaf_size = 0.5;
  float source_voxel_grid_min_dist_in_voxel = 0.1;
  float target_ivox_map_leaf_size = 0.5;
  float target_ivox_map_min_dist_in_voxel = 0.1;
  size_t num_corres_points = 5;
  float max_corres_distance = 2.24;
  float plane_validity_distance = 0.04;
  float lidar_point_noise_std_dev = 0.02;
  bool use_huber = true;
  float huber_threshold = 1.345;
  bool reg_4_dof = false;
  bool project_on_degneneracy = true;
  float degen_thresh_rot = 10;
  float degen_thresh_trans = 15;
};

void declare_config(RegistrationConfig & config);

struct GeometricConfig
{
  std::string logs_directory = "/tmp/";
  bool enabled = true;
  std::string log_level = "info";

  std::string world_frame = "mimosa_world";
  std::string sensor_frame = "mimosa_lidar";
  std::string body_frame = "mimosa_body";
  gtsam::Pose3 T_B_L = gtsam::Pose3::Identity();

  int point_skip_divisor = 1;
  int ring_skip_divisor = 1;
  float map_keyframe_trans_thresh = 0.1;
  float map_keyframe_rot_thresh_deg = 10;
  size_t lru_horizon = 100;
  size_t neighbor_voxel_mode = 7;

  RegistrationConfig scan_to_map;
};

void declare_config(GeometricConfig & config);
}  // namespace lidar
}  // namespace mimosa
