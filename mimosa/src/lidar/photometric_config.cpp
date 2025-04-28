// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/lidar/photometric_config.hpp"

namespace mimosa
{
namespace lidar
{

void declare_config(PhotometricConfig & config)
{
  using namespace config;
  name("Lidar Photometric Config");

  field(config.logs_directory, "logs_directory", "directory_path");
  field(config.body_frame, "body_frame", "str");
  {
    NameSpace ns("lidar");
    field(config.T_B_L, "T_B_S", "gtsam::Pose3");
    field(config.sensor_frame, "sensor_frame", "str");
    {
      NameSpace ns("photometric");
      field(config.log_level, "log_level", "trace|debug|info|warn|error|critical");
      field(config.enabled, "enabled", "bool");
      field(config.destagger, "destagger", "bool");
      field(config.visualize, "visualize", "bool");
      field(config.range_min, "range_min", "float");
      field(config.range_max, "range_max", "float");
      field(config.erosion_buffer, "erosion_buffer", "int");
      field(config.patch_size, "patch_size", "int");
      field(config.margin_size, "margin_size", "int");
      field(config.static_mask_path, "static_mask_path", "file_path");
      field(config.intensity_scale, "intensity_scale", "float");
      field(config.intensity_gamma, "intensity_gamma", "float");
      field(config.remove_lines, "remove_lines", "bool");
      field(config.filter_brightness, "filter_brightness", "bool");
      field(config.gaussian_blur, "gaussian_blur", "bool");
      field(config.gaussian_blur_size, "gaussian_blur_size", "int");
      field(config.gradient_threshold, "gradient_threshold", "float");
      field(config.max_dist_from_mean, "max_dist_from_mean", "float");
      field(config.max_dist_from_plane, "max_dist_from_plane", "float");
      field(config.num_features_detect, "num_features_detect", "size_t");
      field(config.nma_radius, "nma_radius", "int");
      field(config.occlusion_range_diff_threshold, "occlusion_range_diff_threshold", "float");
      field(config.max_feature_life_time, "max_feature_life_time", "int");
      field(config.high_pass_fir, "high_pass_fir", "float[]");
      field(config.low_pass_fir, "low_pass_fir", "float[]");
      field(config.brightness_window_size, "brightness_window_size", "int[]");
      field(
        config.rotate_patch_to_align_with_gradient, "rotate_patch_to_align_with_gradient", "bool");
      field(config.edgelet_patch_offsets, "edgelet_patch_offsets", "vector<int[2]>");
      field(config.use_robust_cost_function, "use_robust_cost_function", "bool");
      field(config.robust_cost_function, "robust_cost_function", "str");
      field(config.robust_cost_function_parameter, "robust_cost_function_parameter", "float");
      field(config.error_scale, "error_scale", "float");
      field(config.sigma, "sigma", "float");
      field(config.max_error, "max_error", "float");
    }

    {
      NameSpace ns("sensor/lidar_data_format");
      field(config.pixel_shift_by_row, "pixel_shift_by_row", "int[]");
      field(config.rows, "pixels_per_column", "int");
      field(config.cols, "columns_per_frame", "int");
    }
    {
      NameSpace ns("sensor/beam_intrinsics");
      field(config.beam_altitude_angles, "beam_altitude_angles", "float[]");
      field(config.lidar_origin_to_beam_origin_mm, "lidar_origin_to_beam_origin_mm", "float");
    }
  }

  check(config.rows, GE, 64, "rows");
  check(config.cols, GE, 512, "cols");
  checkCondition(config.rows * config.cols < INT_MAX, "rows * cols exceeds INT_MAX");
  checkCondition(config.range_min < config.range_max, "range_min < range_max");
  check(config.erosion_buffer, GT, 0, "erosion_buffer");
  check(config.patch_size, GT, 0, "patch_size > 0");
  check(config.patch_size % 2, EQ, 1, "patch_size is odd");
  check(config.pixel_shift_by_row.size(), EQ, config.rows, "pixel_shift_by_row");
  check(config.beam_offset_m, GE, 0.0, "beam_offset_m");
  check(config.intensity_scale, GT, 0.0, "intensity_scale > 0");
  check(config.brightness_window_size.size(), EQ, 2, "brightness_window_size");
  check(config.gaussian_blur_size, GT, 0, "gaussian_blur_size > 0");
  check(config.gaussian_blur_size % 2, EQ, 1, "gaussian_blur_size must be odd");
  checkCondition(
    config.robust_cost_function == "huber" || config.robust_cost_function == "gemanmcclure",
    "robust_cost_function must be huber or gemanmcclure");
  checkCondition(
    config.robust_cost_function == "huber" ? config.robust_cost_function_parameter > 0 : true,
    "robust_cost_function_parameter for huber must be positive");

  // Derived features
  static bool initialized = false;
  if (!initialized) {
    config.fx = -float(config.cols) / (2 * M_PI);
    config.cx = float(config.cols) / 2.0;
    config.fy =
      -float(config.rows) /
      fabs(DEG2RAD(config.beam_altitude_angles.front() - config.beam_altitude_angles.back()));
    config.beam_offset_m = config.lidar_origin_to_beam_origin_mm / 1000.0;

    config.brightness_window_size_cv =
      cv::Size(config.brightness_window_size[0], config.brightness_window_size[1]);

    initialized = true;
  }
}

}  // namespace lidar
}  // namespace mimosa
