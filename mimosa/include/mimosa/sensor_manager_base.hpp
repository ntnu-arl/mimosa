// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa/graph/manager.hpp"

namespace mimosa
{
// Base configuration struct with common fields across all sensor managers
struct SensorManagerBaseConfig
{
  std::string logs_directory = "/tmp/";
  std::string log_level = "info";
  std::string map_frame = "map";
  std::string body_frame = "body";
  std::string sensor_frame = "sensor";
  gtsam::Pose3 T_B_S = gtsam::Pose3::Identity();
  float ts_offset = 0.0;
  bool enabled = true;
  bool use_to_init = true;
  int initial_skip = 0;
};

/**
 * Helper function to declare common config fields for sensor managers
 * This eliminates duplication in declare_config functions
 *
 * @param base_config The base configuration struct to populate
 * @param sensor_namespace The namespace for this sensor (e.g., "lidar", "radar", "odometry")
 */
inline void declare_sensor_manager_config_base(
  SensorManagerBaseConfig & base_config, const std::string & sensor_namespace)
{
  using namespace config;

  field(base_config.logs_directory, "logs_directory", "directory_path");
  field(base_config.map_frame, "map_frame", "str");
  field(base_config.body_frame, "body_frame", "str");

  {
    NameSpace ns(sensor_namespace);
    field(base_config.sensor_frame, "sensor_frame", "str");
    field(base_config.T_B_S, "T_B_S", "gtsam::Pose3");
    {
      NameSpace ns("manager");
      field(base_config.log_level, "log_level", "trace|debug|info|warn|error|critical");
      field(base_config.enabled, "enabled", "bool");
      field(base_config.use_to_init, "use_to_init", "bool");
      field(base_config.initial_skip, "initial_skip", "number of messages to drop at the start");
      field(base_config.ts_offset, "ts_offset", "s");
    }
  }

  // Common validation
  check(base_config.initial_skip, GE, 0, "initial_skip");
  check(base_config.body_frame, NE, base_config.map_frame, "body_frame");
  check(
    base_config.sensor_frame, NE, base_config.map_frame,
    "sensor_frame should not be the same as map_frame");
  check(
    base_config.sensor_frame, NE, base_config.body_frame,
    "sensor_frame should not be the same as body_frame");
}

/**
 * Base class for sensor managers providing common functionality
 * Template parameters:
 *   ConfigT - The sensor-specific configuration type
 *   MsgT - The ROS message type for this sensor
 */
template <typename ConfigT, typename MsgT>
class SensorManagerBase
{
protected:
  // Common member variables
  const ConfigT config_;
  std::unique_ptr<spdlog::logger> logger_;
  ros::Subscriber sub_;
  tf2_ros::StaticTransformBroadcaster tf2_static_broadcaster_;
  mimosa::imu::Manager::Ptr imu_manager_;
  mimosa::graph::Manager::Ptr graph_manager_;

  // Common timing variables
  gtsam::Key new_key_;
  bool initialized_ = false;
  const std::string manager_type_;
  double header_ts_;
  double
    corrected_ts_;  // The mechanism to get the corrected timestamp is sensor-specific and is implemented in the derived class

  int initial_skip_;
  double prev_ts_ = 0.0;

public:
  inline std::string getSubscribedTopic() const { return sub_.getTopic(); }

protected:
  SensorManagerBase(
    const ConfigT & config, mimosa::imu::Manager::Ptr imu_manager,
    mimosa::graph::Manager::Ptr graph_manager, const std::string & manager_type)
  : config_(config),
    imu_manager_(imu_manager),
    graph_manager_(graph_manager),
    manager_type_(manager_type)
  {
    logger_ = createLogger(
      config_.base.logs_directory + manager_type_ + "_manager.log", manager_type_ + "::Manager",
      config_.base.log_level);
    logger_->info("Initialized with params:\n {}", config::toString(config_));

    initial_skip_ = config_.base.initial_skip;
  }

  virtual void callback(const typename MsgT::ConstPtr & msg) = 0;

  inline void subscribeIfEnabled(ros::NodeHandle & pnh)
  {
    if (isEnabled()) {
      sub_ = pnh.subscribe<MsgT>(
        manager_type_ + "/manager/" + manager_type_ + "_in", 5, &SensorManagerBase::callback, this);
    }
  }

  inline void broadcastStaticTransform(double timestamp)
  {
    broadcastTransform(
      tf2_static_broadcaster_, config_.base.T_B_S, config_.base.body_frame,
      config_.base.sensor_frame, timestamp);
  }

  inline bool isEnabled() const { return config_.base.enabled; }

  inline bool isImuReady() const
  {
    // This could be simpler but is done this way to provide more informative logging
    if (!imu_manager_->hasRecievedFirstMessage()) {
      logger_->debug("imu::Manager has not received the first message yet. Skipping this message.");
      return false;
    }
    return true;
  }

  inline bool handleInitialSkip()
  {
    if (initial_skip_ > 0) {
      --initial_skip_;
      logger_->info("Dropping initial message. {} more will be dropped", initial_skip_);
      return false;  // Should skip
    }
    return true;  // Don't skip
  }

  inline bool validateTimestamp(const typename MsgT::ConstPtr & msg)
  {
    header_ts_ = msg->header.stamp.toSec();

    if (header_ts_ <= prev_ts_) {
      logger_->error(
        "Skipping message with timestamp {} because it is not newer than previous timestamp {}. "
        "This should never happen. Check the header timestamps of the incoming messages",
        header_ts_, prev_ts_);
      return false;
    }

    prev_ts_ = header_ts_;
    return true;
  }

  inline bool validateMessage(const typename MsgT::ConstPtr & msg)
  {
    // For a pointcloud the only check is that the data is not empty since actual checks by parsing are handled in the individual managers
    if constexpr (std::is_same<MsgT, sensor_msgs::PointCloud2>::value) {
      if (msg->data.empty()) {
        logger_->warn("Received empty message. Ignoring this measurement");
        return false;
      }
    }
    // For odometry we can check that the values
    if constexpr (std::is_same<MsgT, nav_msgs::Odometry>::value) {
      if (
        std::isnan(msg->pose.pose.position.x) || std::isnan(msg->pose.pose.position.y) ||
        std::isnan(msg->pose.pose.position.z)) {
        logger_->warn("Received odometry message with NaN position. Ignoring this measurement");
        return false;
      }
      if (
        std::isnan(msg->pose.pose.orientation.x) || std::isnan(msg->pose.pose.orientation.y) ||
        std::isnan(msg->pose.pose.orientation.z) || std::isnan(msg->pose.pose.orientation.w)) {
        logger_->warn("Received odometry message with NaN orientation. Ignoring this measurement");
        return false;
      }
    }
    return true;
  }

  // If true, this function will set the value of header_ts_
  inline bool passesCommonValidations(const typename MsgT::ConstPtr & msg)
  {
    return isEnabled() && isImuReady() && handleInitialSkip() && validateTimestamp(msg) &&
           validateMessage(msg);
  }

  bool handleDeclarationResult(graph::Manager::DeclarationResult result)
  {
    switch (result) {
      case graph::Manager::DeclarationResult::FAILURE_CANNOT_INIT_ON_MODALITY:
        logger_->debug("Cannot initialize on this modality. Skipping this message");
        return false;

      case graph::Manager::DeclarationResult::FAILURE_ATTITUDE_ESTIMATION:
        logger_->debug(
          "Graph manager could not initialize due to attitude estimation failure. Skipping this "
          "message");
        return false;

      case graph::Manager::DeclarationResult::FAILURE_OLDER_THAN_INITIALIZATION:
        logger_->info(
          "timestamp is older than the graph initialization timestamp. If this "
          "happens continuously, check the time synchronization between the sensors");
        return false;

      case graph::Manager::DeclarationResult::FAILURE_OLDER_THAN_LAG:
        logger_->warn("Timestamp is older than entire lag window. Ignoring measurement");
        return false;

      case graph::Manager::DeclarationResult::FAILURE_OLDER_THAN_MAX_LATENCY:
        logger_->warn("Timestamp is older than max latency. Ignoring measurement");
        return false;

      case graph::Manager::DeclarationResult::FAILURE_CANNOT_HANDLE_OUT_OF_ORDER:
        logger_->error(
          "Was not able to find the IMU factor to break. This should never happen. Contact "
          "developers with your rosbag");
        return false;

      case graph::Manager::DeclarationResult::SUCCESS_INITIALIZED:
      case graph::Manager::DeclarationResult::SUCCESS_SAME_KEY:
      case graph::Manager::DeclarationResult::SUCCESS_OUT_OF_ORDER:
      case graph::Manager::DeclarationResult::SUCCESS_NORMAL:
        return true;

      default:
        logCriticalException<std::logic_error>(
          logger_, fmt::format(
                     "Unknown declaration result. Maybe the graph::manager::DeclarationResult API "
                     "has been updated. Received: {}",
                     result));
        return false;
    }
  }
};

}  // namespace mimosa
