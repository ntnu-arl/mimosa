// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa/sensor_manager_base.hpp"
#include "mimosa/lidar/geometric.hpp"
#include "mimosa/lidar/photometric.hpp"
#include "mimosa_msgs/LidarManagerDebug.h"

// pcl_conversions
#include <pcl_conversions/pcl_conversions.h>

namespace mimosa
{
namespace lidar
{
struct ManagerConfig
{
  SensorManagerBaseConfig base;

  // Lidar-specific fields
  gtsam::Pose3 T_B_OdometryLoggerFrame = gtsam::Pose3::Identity();
  bool transpose_pointcloud = false;
  float range_min = 0.0;
  float range_max = 100.0;
  float intensity_min = 0.0;
  float intensity_max = 1e10;
  float ns_max = 1e9;
  std::vector<float> lidar_to_sensor_transform = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  bool create_full_res_pointcloud = false;
  int full_res_pointcloud_publish_rate_divisor = 1;
  bool use_reflectivity_as_intensity = false;
  bool scale_intensity_by_sq_range = false;
  bool near_range_correction = false;
};

void declare_config(ManagerConfig & config);

class Manager : public SensorManagerBase<ManagerConfig, sensor_msgs::PointCloud2>
{
private:
  std::unique_ptr<spdlog::logger> trajectory_logger_;

  // Member variables
  std::unique_ptr<Geometric> geometric_;
  std::unique_ptr<Photometric> photometric_;
  pcl::PointCloud<Point> points_full_;
  pcl::PointCloud<Point> points_raw_;
  std::vector<size_t> geometric_point_idxs_;
  boost::container::flat_map<uint32_t, gtsam::Pose3> interpolated_map_T_Le_Lt_;
  std::vector<std::pair<uint32_t, size_t>> ns_idx_pairs_;
  std::vector<uint32_t> unique_ns_;
  std::vector<std::vector<size_t>> idxs_at_unique_ns_;

  const float z_offset_;
  const float range_min_sq_;
  const float range_max_sq_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_preintegrator_;
  gtsam::NavState propagated_state_;
  mimosa_msgs::LidarManagerDebug debug_msg_;
  M66 geometric_eigenvectors_block_matrix_ = M66::Identity();
  V6D geometric_degen_directions_ = V6D::Ones();

  State prev_state_;

  // Outputs
  ros::Publisher pub_points_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_path_;

public:
  Manager(
    ros::NodeHandle & pnh, mimosa::imu::Manager::Ptr imu_manager,
    mimosa::graph::Manager::Ptr graph_manager);
  void callback(const sensor_msgs::PointCloud2::ConstPtr & msg) override;

private:
  template <typename PointT>
  void prepareInput(const sensor_msgs::PointCloud2::ConstPtr & msg);
  void deskewPoints();
  void preprocess(const gtsam::Key key);
  void getFactors(const gtsam::Values & initial_values, gtsam::NonlinearFactorGraph & new_factors);
  void define(
    const gtsam::NonlinearFactorGraph & new_factors, gtsam::Values & optimized_values,
    const graph::Manager::DeclarationResult result);
  void postDefineUpdate(const gtsam::Key key, const gtsam::Values & values);
  void publishResults(const gtsam::Pose3 & T_W_Bk_opt);
  inline double globalTs(const uint32_t & value) const { return header_ts_ + value * 1.0e-9; }
};

}  // namespace lidar
}  // namespace mimosa
