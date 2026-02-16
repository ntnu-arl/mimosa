// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa/odometry/utils.hpp"
#include "mimosa/sensor_manager_base.hpp"

namespace mimosa
{
namespace odometry
{
struct ManagerConfig
{
  SensorManagerBaseConfig base;

  // Odometry-specific fields
  float d_opt_thresh = 1;
  float sigma_rot_deg = 1.0;
  float sigma_trans_m = 0.5;
};

void declare_config(ManagerConfig & config);

class Manager : public SensorManagerBase<ManagerConfig, nav_msgs::Odometry>
{
private:
  gtsam::Key prev_key_;
  gtsam::Pose3 T_Ow_Skm1_;

public:
  Manager(
    ros::NodeHandle & pnh, mimosa::imu::Manager::Ptr imu_manager,
    mimosa::graph::Manager::Ptr graph_manager);
  void callback(const nav_msgs::Odometry::ConstPtr & msg) override;
};

}  // namespace odometry
}  // namespace mimosa
