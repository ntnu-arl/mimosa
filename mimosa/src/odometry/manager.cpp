// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/odometry/manager.hpp"

namespace mimosa
{
namespace odometry
{
Manager::Manager(
  ros::NodeHandle & pnh, mimosa::imu::Manager::Ptr imu_manager,
  mimosa::graph::Manager::Ptr graph_manager)
: SensorManagerBase<ManagerConfig, nav_msgs::Odometry>(
    config::checkValid(config::fromRos<ManagerConfig>(pnh)), imu_manager, graph_manager, "odometry")
{
  subscribeIfEnabled(pnh);
}

void Manager::callback(const nav_msgs::Odometry::ConstPtr & msg)
{
  Stopwatch sw;
  if (!passesCommonValidations(msg)) {
    return;
  }
  corrected_ts_ = msg->header.stamp.toSec() + config_.base.ts_offset;

  MXD cov;
  convert(msg->pose.covariance, cov);
  // Calculate D optimality Metric
  double d_opt = calcDoptimality(cov);

  logger_->debug("d opt value {}", d_opt);

  // Threshold d_optimality
  if (d_opt > config_.d_opt_thresh) {
    logger_->error(
      "D opt value {} is higher than threshold {}", d_opt,
      config_.d_opt_thresh);
    return;
  }

  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Pose3 T_Ow_Sk = toGtsam(msg->pose.pose);
  if (initialized_) {
    gtsam::Pose3 T_Bkm1_Bk = config_.base.T_B_S * T_Ow_Skm1_.inverse() * T_Ow_Sk * config_.base.T_B_S.inverse();
    gtsam::noiseModel::Diagonal::shared_ptr noise_model = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << deg2rad(config_.sigma_rot_deg), deg2rad(config_.sigma_rot_deg),
       deg2rad(config_.sigma_rot_deg), config_.sigma_trans_m, config_.sigma_trans_m,
       config_.sigma_trans_m)
        .finished());
    // The 0 key is a special key that gets mapped to the value of new_key_ in graph_manager_->declare()
    new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(X(prev_key_), X(0), T_Bkm1_Bk, noise_model));
  }

  graph::Manager::DeclarationResult dr =
    graph_manager_->declare(corrected_ts_, new_key_, config_.base.use_to_init, new_factors);

  if (!handleDeclarationResult(dr)) {
    return;
  }

  T_Ow_Skm1_ = T_Ow_Sk;
  prev_key_ = new_key_;
  initialized_ = true;
}

void declare_config(ManagerConfig & config)
{
  using namespace config;
  name("Odometry Manager Config");

  // Declare common sensor manager config fields
  declare_sensor_manager_config_base(config.base, "odometry");

  // Declare odometry-specific fields
  {
    NameSpace ns("odometry");
    {
      NameSpace ns("manager");
      field(config.d_opt_thresh, "d_opt_thresh", "D-optimality threshold");
      field(config.sigma_rot_deg, "sigma_rot_deg", "deg");
      field(config.sigma_trans_m, "sigma_trans_m", "m");
    }
  }
}

}  // namespace odometry
}  // namespace mimosa
