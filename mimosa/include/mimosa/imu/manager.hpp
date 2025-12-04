// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa/state.hpp"
#include "mimosa/stopwatch.hpp"
#include "mimosa/utils.hpp"

// mimosa_msgs
#include "mimosa_msgs/ImuManagerDebug.h"

// ROS
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

// GTSAM
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

// C++
#include <deque>

namespace mimosa
{
typedef std::pair<double, V6D> ImuMeasurement; // < time, (acc, omega)> 
typedef std::deque<ImuMeasurement> ImuBuffer;

namespace imu
{
struct PreintegrationConfig
{
  float acc_noise_density = 0.0013886655606357616;
  float acc_bias_random_walk = 8.538212723310593e-05;
  float gyro_noise_density = 5.4380545102010436e-05;
  float gyro_bias_random_walk = 1.6587925152480572e-06;
  float integration_sigma = 1.0e-4;
  bool use_2nd_order_coriolis = false;
  bool use_estimated_gravity = false;
  float gravity_magnitude = 9.81;
  bool gravity_aligned_initialization = true;
};

void declare_config(PreintegrationConfig & config);

struct ManagerConfig
{
  std::string logs_directory = "/tmp/";
  std::string log_level = "info";
  std::string world_frame = "mimosa_world";
  std::string body_frame = "mimosa_body";
  std::string sensor_frame = "mimosa_imu";
  gtsam::Pose3 T_B_S = gtsam::Pose3::Identity();
  float ts_offset = 0.0;                   // s
  float max_buffer_duration = 2.0;         // s
  float pose_init_wait_secs = 1.0;         // s
  float interpolation_max_ts_diff = 0.01;  // s
  float extrapolation_max_ts_diff = 0.01;  // s
  float acc_scale_factor = 1.0;            // used only when IMU measurements are in units of g

  PreintegrationConfig preintegration;
};

void declare_config(ManagerConfig & config);

class Manager
{
private:
  const ManagerConfig config_;
  std::unique_ptr<spdlog::logger> logger_;

  // Inputs
  ros::Subscriber sub_;

  // Member variables
  ImuBuffer buffer_;
  std::mutex buffer_mutex_;
  bool has_recieved_first_message_ = false;
  std::shared_ptr<gtsam::PreintegrationParams> preintegrator_params_;
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> preintegrator_;
  std::mutex preintegrator_mutex_;
  mimosa_msgs::ImuManagerDebug debug_msg_;

  // Propagation
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> propagation_preintegrator_;
  State propagation_base_state_;
  double propagated_upto_ts_;
  std::mutex propagation_mutex_;

  // Outputs
  tf2_ros::StaticTransformBroadcaster tf2_static_broadcaster_;
  ros::Publisher pub_debug_;
  ros::Publisher pub_localizability_marker_array_;
  ros::Publisher pub_odometry_;

  void updatePreintegrationTo(
    const double ts_start, const double ts_end, const gtsam::imuBias::ConstantBias & imu_bias,
    std::unique_ptr<gtsam::PreintegratedImuMeasurements> & preintegrator);
  std::shared_ptr<gtsam::PreintegrationParams> generatePreintegratorParams() const;
  std::shared_ptr<gtsam::PreintegrationParams> generatePreintegratorParams(
    const V3D & gravity) const;
  void addImuFactorNoLock(
    const double ts_0, const double ts_1, const gtsam::imuBias::ConstantBias & bias_0,
    const gtsam::Key key_0, const gtsam::Key key_1, gtsam::NonlinearFactorGraph & graph);
  V6D interpolateMeasurement(
    const double ts1, const V6D & meas1, const double ts2, const V6D & meas2, const double ts);

public:
  using Ptr = std::shared_ptr<Manager>;
  Manager(ros::NodeHandle & pnh);
  // * This function is public to allow other sensor managers to create thier own preintegrators if needed
  inline std::shared_ptr<gtsam::PreintegrationParams> getPreintegratorParams()
  {
    return preintegrator_params_;
  }
  inline bool hasRecievedFirstMessage() const { return has_recieved_first_message_; }
  void callback(const sensor_msgs::Imu::ConstPtr & msg);
  bool estimateAttitude(gtsam::Rot3 & R_W_B, V3D & estimated_acc_bias, V3D & estimated_gyro_bias);
  void getInterpolatedMeasurements(
    const double ts_start, const double ts_end, ImuBuffer & measurements,
    const bool dont_interpolate_first_measurement = false);
  size_t getNumMeasurementsBetween(const double t1, const double t2);
  void addImuFactor(
    const double ts_0, const double ts_1, const gtsam::imuBias::ConstantBias & bias_0,
    const gtsam::Key key_0, const gtsam::Key key_1, gtsam::NonlinearFactorGraph & graph);
  void addImuFactorAndGetNavState(
    const State & state_0, const double ts_1, const gtsam::Key key_1,
    gtsam::NonlinearFactorGraph & graph, gtsam::NavState & ns_1);

  const ManagerConfig & config() const { return config_; }
  void setPropagationBaseState(const State & state);
  inline std::string getSubscribedTopic() const { return sub_.getTopic(); }
};

}  // namespace imu
}  // namespace mimosa
