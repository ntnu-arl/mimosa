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
struct PhotometricConfig
{
  std::string logs_directory = "/tmp/";
  std::string log_level = "info";

  bool enabled = true;
  bool destagger = true;
  bool visualize = false;
  std::string sensor_frame = "mimosa_lidar";
  std::string body_frame = "mimosa_body";

  // LiDAR config
  std::vector<int> pixel_shift_by_row = {};

  gtsam::Pose3 T_B_L = gtsam::Pose3::Identity();

  size_t rows = 128;
  size_t cols = 512;
  float range_min = 0.1;
  float range_max = 100;
  int erosion_buffer = 2;
  int patch_size = 5;
  int margin_size = 2;
  std::string static_mask_path = "";
  float intensity_scale = 0.25;
  float intensity_gamma = 0.8;
  bool remove_lines = true;
  bool filter_brightness = true;
  bool gaussian_blur = true;
  int gaussian_blur_size = 3;
  float gradient_threshold = 20;
  float max_dist_from_mean = 0.2;
  float max_dist_from_plane = 0.1;
  int nma_radius = 10;
  size_t num_features_detect = 60;
  float occlusion_range_diff_threshold = 0.1;
  int max_feature_life_time = 30;
  std::vector<float> beam_altitude_angles = {};
  std::vector<double> high_pass_fir = {};
  std::vector<double> low_pass_fir = {};
  std::vector<int> brightness_window_size = {};
  float lidar_origin_to_beam_origin_mm = 0.0;

  // Edgelets
  // Default is full 5x5 patch. The locations of u,v + offset are used
  bool rotate_patch_to_align_with_gradient = false;
  std::vector<std::pair<int, int>> edgelet_patch_offsets = {
    {-2, -2}, {-1, -2}, {0, -2}, {1, -2}, {2, -2}, {-2, -1}, {-1, -1}, {0, -1}, {1, -1},
    {2, -1},  {-2, 0},  {-1, 0}, {0, 0},  {1, 0},  {2, 0},   {-2, 1},  {-1, 1}, {0, 1},
    {1, 1},   {2, 1},   {-2, 2}, {-1, 2}, {0, 2},  {1, 2},   {2, 2},
  };

  // Factor
  bool use_robust_cost_function = true;
  std::string robust_cost_function = "huber";
  double robust_cost_function_parameter = 1.345;
  double error_scale = 1.0;
  double max_error = 255.0;
  double sigma = 0.1;

  // Derived parameters
  double fx = 1.0;
  double fy = 1.0;
  double cx = 0.0;
  float beam_offset_m = 0.0;
  cv::Size brightness_window_size_cv = cv::Size(1, 1);
  double sigma_azimuth = 0.001;
  double sigma_elevation = 0.001;
};

void declare_config(PhotometricConfig & config);

}  // namespace lidar
}  // namespace mimosa
