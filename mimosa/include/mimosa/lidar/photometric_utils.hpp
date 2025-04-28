// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <boost/container/flat_map.hpp>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

#include "mimosa/lidar/photometric_config.hpp"

#define DUPLICATE_POINTS 10

namespace mimosa
{
namespace lidar
{

using CvMatPtr = std::shared_ptr<cv::Mat>;
using CvMatConstPtr = std::shared_ptr<const cv::Mat>;

struct Feature
{
  uint32_t id;                      // Feature ID
  gtsam::Key key;                   // Key of the pose that the feature was detected in
  int life_time;                    // num of tracking
  V2D center;                       // Center Pixel coordinate in last frame
  std::vector<double> intensities;  // Pixel intensities
  std::vector<V3D> Le_ps;           // 3D positions in the frame that the feature was detected in
  std::vector<V2D> uvs;             // Pixel coordinates in last frame
  V3D
    normal;  // Normal of the feature in the frame that the feature was detected in and pointing towards the camera
  std::vector<gtsam::Pose3>
    T_Le_Lts;  // Transform from the Lt frame to the Le frame. Note these are in the pointcloud that the feature was detected in and allow the points to be skewed.
  VXD psi_intensities;     // Normalized intensities
  double mean_intensity;   // Mean intensity
  double sigma_intensity;  // Standard deviation of the intensities
};

struct Frame
{
  using Ptr = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

  double ts;
  gtsam::Key key;
  pcl::PointCloud<Point> points_deskewed;
  cv::Mat img_intensity;
  cv::Mat img_range;
  cv::Mat img_deskewed_cloud_idx;
  std::vector<int> proj_idx;
  boost::container::flat_map<uint32_t, gtsam::Pose3> interpolated_map_T_Le_Lt;
  cv::Mat img_dx;
  cv::Mat img_dy;
  cv::Mat img_mask;
  std::vector<std::vector<float>> yaw_angles;
  bool is_keyframe;

  Frame(
    const size_t rows, const size_t cols, const double ts_orig, const gtsam::Key key_orig,
    const pcl::PointCloud<Point> & points_deskewed_orig,
    const boost::container::flat_map<uint32_t, gtsam::Pose3> & interpolated_map_T_Le_Lt_orig)
  {
    ts = ts_orig;
    key = key_orig;
    points_deskewed = points_deskewed_orig;
    interpolated_map_T_Le_Lt = interpolated_map_T_Le_Lt_orig;
    img_intensity = cv::Mat::zeros(rows, cols, CV_32FC1);
    img_range = cv::Mat::zeros(rows, cols, CV_32FC1);
    img_deskewed_cloud_idx = cv::Mat::ones(rows, cols, CV_32SC1) * (-1);
    proj_idx = std::vector<int>(rows * cols * DUPLICATE_POINTS, 0);
    img_mask = cv::Mat::zeros(rows, cols, CV_8UC1);  // zero: invalid, one: valid
    img_dx = cv::Mat::zeros(rows, cols, CV_32F);
    img_dy = cv::Mat::zeros(rows, cols, CV_32F);
    yaw_angles.resize(rows);
    for (auto & row : yaw_angles) {
      // Starting with NaNs so that the program will crash if we try to access an invalid index
      row.resize(cols, std::numeric_limits<float>::quiet_NaN());
    }
    is_keyframe = false;
  }
};

void getPsi(const VXD & i_bar, double & mean, double & sigma, VXD & psi);
void getPsiJacobian(const double sigma, const VXD & psi, MXD & H);
bool inFOV(const V2D & uv, const PhotometricConfig & config);
bool project(const V3D & p, V2D & uv, const PhotometricConfig & config);
bool project(
  const V3D & p, V2D & uv, const std::vector<std::vector<float>> & yaw_angles,
  const PhotometricConfig & config, const bool print = false);
void getProjectionJacobian(const V3D & Li_p, const PhotometricConfig & config, M23 & H);
int vectorIndexFromRowCol(const int row, const int col, const PhotometricConfig & config);
bool projectUndistorted(
  const V3D & Le_p, V3D & Li_p, V2D & Li_uv, int & distortion_idx, const bool round,
  const std::vector<int> & proj_idx, const pcl::PointCloud<Point> & points_deskewed,
  const boost::container::flat_map<uint32_t, gtsam::Pose3> & interpolated_map_T_Le_Lt_,
  const PhotometricConfig & config);
bool projectUndistorted(
  const V3D & Le_p, V3D & Li_p, V2D & Li_uv, gtsam::Pose3 & T_Le_Lt, const bool round,
  const std::vector<int> & proj_idx, const pcl::PointCloud<Point> & points_deskewed,
  const boost::container::flat_map<uint32_t, gtsam::Pose3> & interpolated_map_T_Le_Lt_,
  const std::vector<std::vector<float>> & yaw_angles, const PhotometricConfig & config);
double getSubPixelValue(const cv::Mat & img, const double x, const double y);
double getSubPixelValue(const cv::Mat & img, const V2D loc_uv);
void drawFeatures(
  const cv::Mat & img_gray_u8, const std::vector<Feature> & features, cv::Mat & img_with_features,
  const std::vector<double> & print_vals = {});
void visualize(const cv::Mat & image, const std::string & window_name);
void visualize(const std::vector<std::vector<float>> & v, const std::string & window_name);
// Helper function: Snap a continuous point (x,y) to the nearest grid point,
// resolving collisions by checking the 8-neighborhood.
std::pair<int, int> snapPoint(
  const std::pair<double, double> & p, std::set<std::pair<int, int>> & used);
// Function to get grid locations based on the gradient (grad_x, grad_y)
// and an original pattern of points (relative to the pattern center).
std::vector<std::pair<int, int>> getGradientBasedLocations(
  const float grad_x, const float grad_y,
  const std::vector<std::pair<int, int>> & original_pattern);

}  // namespace lidar
}  // namespace mimosa
