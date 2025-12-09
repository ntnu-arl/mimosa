// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa/imu/manager.hpp"
#include "mimosa/state.hpp"
#include "mimosa/stopwatch.hpp"
#include "mimosa_msgs/GraphManagerDebug.h"

// ROS
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

// GTSAM
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#define SMOOTHER_IFL true
namespace mimosa
{
namespace graph
{
struct SmootherConfig
{
  float lag = 0.25;  // s
  float wildfire_threshold = 0.001;
  float relinearize_threshold_translation = 0.1;
  float relinearize_threshold_rotation = 0.1;
  float relinearize_threshold_velocity = 0.1;
  float relinearize_threshold_bias_acc = 0.1;
  float relinearize_threshold_bias_gyro = 0.1;
  float relinearize_threshold_gravity = 0.1;
  int relinearize_skip = 1;
  bool enable_relinearization = true;
  bool evaluate_nonlinear_error = false;
  std::string factorization = "QR";
  bool cache_linearized_factors = true;
  bool enable_detailed_results = false;
  bool enable_partial_relinearization_check = false;
  float initial_position_sigma = 0.1;
  float initial_rotation_yaw_sigma_deg = 1.0;
  float initial_rotation_pitch_roll_sigma_deg = 1.0;
  float initial_velocity_sigma = 0.1;
  float initial_bias_acc_sigma = 0.1;
  float initial_bias_gyro_sigma = 0.1;
  float initial_gravity_sigma = 0.1;
  int additional_update_iterations = 0;
};

void declare_config(SmootherConfig & config);

struct ManagerConfig
{
  std::string world_frame = "world";
  std::string navigation_frame = "navigation";
  std::string body_frame = "body";
  std::string logs_directory = "/tmp/";
  std::string log_level = "info";
  float max_measurement_latency = 0.1;          // s
  SmootherConfig smoother;
};

void declare_config(ManagerConfig & config);

class Manager
{
public:
  using Ptr = std::shared_ptr<Manager>;

  enum class DeclarationResult
  {
    FAILURE_CANNOT_INIT_ON_MODALITY = 0,
    FAILURE_ATTITUDE_ESTIMATION,
    FAILURE_OLDER_THAN_INITIALIZATION,
    FAILURE_OLDER_THAN_LAG,
    FAILURE_OLDER_THAN_MAX_LATENCY,
    FAILURE_CANNOT_HANDLE_OUT_OF_ORDER,
    SUCCESS_INITIALIZED,
    SUCCESS_SAME_KEY,
    SUCCESS_OUT_OF_ORDER,
    SUCCESS_NORMAL,
  };

private:
  const ManagerConfig config_;
  std::unique_ptr<spdlog::logger> logger_;
  std::unique_ptr<spdlog::logger> trajectory_logger_;

  // Member variables
  mimosa::imu::Manager::Ptr imu_manager_;
  const gtsam::ISAM2Params isam2_params_;
  bool initialized_;

  State state_;
  gtsam::Key internal_key_;

#if SMOOTHER_IFL
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
#else
  std::unique_ptr<gtsam::ISAM2> smoother_;
#endif
  gtsam::Values optimized_values_;
  std::map<double, gtsam::Key> ts_key_map_;
  std::mutex graph_mutex_;
  mimosa_msgs::GraphManagerDebug debug_msg_;

  // Outputs
  tf2_ros::TransformBroadcaster tf2_broadcaster_;
  tf2_ros::StaticTransformBroadcaster tf2_static_broadcaster_;
  ros::Publisher pub_path_;
  ros::Publisher pub_odometry_;
  ros::Publisher pub_transform_stamped_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_optimized_path_;

public:
  Manager(ros::NodeHandle & pnh, mimosa::imu::Manager::Ptr imu_manager);
  // The one step factors are for things that do not require the two step process. Eg. Radar
  DeclarationResult declare(
    const double ts, gtsam::Key & key, const bool use_to_init,
    const gtsam::NonlinearFactorGraph & one_step_factors = {});
  void getCurrentState(State & state);
  void getStateUpto(const double ts, State & state);
  void define(
    const gtsam::NonlinearFactorGraph & graph, gtsam::Values & optimized_values,
    const DeclarationResult result);
  inline gtsam::Pose3 getPoseAt(const gtsam::Key key)
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return optimized_values_.at<gtsam::Pose3>(X(key));
  }
  inline gtsam::Values getCurrentOptimizedValues()
  {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    return optimized_values_;
  }
  inline const gtsam::NonlinearFactorGraph & getFactors() const
  {
#if SMOOTHER_IFL
    return smoother_->getFactors();
#else
    return smoother_->getFactorsUnsafe();
#endif
  }

private:
  gtsam::ISAM2Params getISAM2Params() const;
  void initializeGraph(
    const double ts, const gtsam::Key key, const gtsam::Pose3 & T_W_B, const V3D & vel,
    const gtsam::imuBias::ConstantBias & bias,
    const gtsam::NonlinearFactorGraph & additional_factors = {});
  void getStateUptoNoLock(const double ts, State & state);
  void rekeyOneStepFactors(
    const gtsam::NonlinearFactorGraph & one_step_factors, const gtsam::Key new_key,
    gtsam::NonlinearFactorGraph & rekeyed_one_step_factors);
  void defineNoLock(
    const gtsam::NonlinearFactorGraph & graph, gtsam::Values & optimized_values,
    const DeclarationResult result);
  std::vector<double> getSmootherTimestampsNoLock();
  void updateStateToKeyTs(const gtsam::Key key, const double ts);
  void publishResults();
  inline gtsam::Key getNextKey() { return internal_key_++; }
  inline void discardCurrentKey() { internal_key_--; }
  inline void resetKey() { internal_key_ = 1; }
};

}  // namespace graph
}  // namespace mimosa
