// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

// mimosa
#include "mimosa/lidar/photometric_config.hpp"
#include "mimosa/lidar/photometric_factor.hpp"
#include "mimosa/lidar/photometric_utils.hpp"
#include "mimosa/lidar/utils.hpp"
#include "mimosa/state.hpp"
#include "mimosa/stopwatch.hpp"
#include "mimosa/utils.hpp"
#include "mimosa_msgs/LidarPhotometricDebug.h"

// C++
#include <algorithm>

namespace mimosa
{
namespace lidar
{
class Photometric
{
public:
  const PhotometricConfig config;

private:
  std::unique_ptr<spdlog::logger> logger_;

  // Member variables
  const cv::Mat erosion_kernel_;
  const cv::Mat mask_margin_;
  const std::vector<int> idx_to_u_;
  const std::vector<int> idx_to_v_;
  std::vector<int> idx_to_uk_;
  std::vector<int> idx_to_vk_;

  cv::Mat static_mask_;

  Frame::Ptr current_frame_;
  std::vector<Feature> map_Le_features_;
  PhotometricFactor::Ptr photometric_factor_;

  mimosa_msgs::LidarPhotometricDebug debug_msg_;

  ros::Publisher pub_img_intensity_;
  ros::Publisher pub_img_new_features_;
  ros::Publisher pub_img_tracked_keyframe_features_;
  ros::Publisher pub_img_mask_;

  ros::Publisher pub_features_;
  ros::Publisher pub_feature_marker_;
  ros::Publisher pub_feature_marker_array_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_localizability_marker_array_;

  uint32_t monotonic_feature_id_ = 0;

public:
  Photometric(ros::NodeHandle & pnh);
  void preprocess(
    const pcl::PointCloud<Point> & points_raw, pcl::PointCloud<Point> & points_deskewed,
    const boost::container::flat_map<uint32_t, gtsam::Pose3> & interpolated_map_T_Le_Lt,
    const double ts, const gtsam::Key key);
  void getFactors(
    const gtsam::Values & values, gtsam::NonlinearFactorGraph & graph,
    const M66 & eigenvectors_block_matrix = M66::Identity(), const V6D & selection = V6D::Ones());
  void updateMap(const gtsam::Values & values, const std::vector<V3D> & bias_directions = {});

private:
  std::vector<int> getIdxToPixelMap(int axis) const;
  void removeLines(cv::Mat & img);
  void filterBrightness(cv::Mat & img);
  void createMask();
  void detectFeatures(
    const int num_to_detect, Frame::ConstPtr frame, std::vector<Feature> & features,
    const gtsam::Pose3 & T_W_Be, const std::vector<V3D> & bias_directions = {});
  void publishFeatures(
    Frame::ConstPtr frame, const gtsam::Values & values, const std::string & map_frame,
    const double ts);
};
}  // namespace lidar
}  // namespace mimosa
