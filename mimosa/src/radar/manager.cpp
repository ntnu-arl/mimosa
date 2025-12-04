// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/radar/manager.hpp"

namespace mimosa
{
namespace radar
{
Manager::Manager(
  ros::NodeHandle & pnh, mimosa::imu::Manager::Ptr imu_manager,
  mimosa::graph::Manager::Ptr graph_manager)
: SensorManagerBase<ManagerConfig, sensor_msgs::PointCloud2>(
    config::checkValid(config::fromRos<ManagerConfig>(pnh)), imu_manager, graph_manager, "radar")
{
  pub_debug_ = pnh.advertise<mimosa_msgs::RadarManagerDebug>("radar/manager/debug", 1);
  pub_filtered_points_ =
    pnh.advertise<sensor_msgs::PointCloud2>("radar/manager/filtered_points", 1);

  subscribeIfEnabled(pnh);
}

void Manager::callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  Stopwatch sw;
  if (!passesCommonValidations(msg)) {
    return;
  }

  // The timestamp should always be in the center of the exposure. If it is not, then this should be compensated for.
  corrected_ts_ = header_ts_ + (config_.is_exposure_compensated ? 0 : config_.frame_ms * 1e-3 / 2);
  logger_->debug("Header timestamp: {}, Corrected timestamp: {}", header_ts_, corrected_ts_);

  static bool first = true;
  if (first) {
    first = false;
    broadcastStaticTransform(corrected_ts_);
  }

  static const PType point_type = decodePointType(msg->fields);
  switch (point_type) {
    case PType::Rio:
      preprocess<rioPoint>(msg);
      break;
    case PType::mmWave:
      preprocess<mmWavePoint>(msg);
      break;
    default:
      throw std::runtime_error("Unsupported point type");
  }

  logger_->trace("Getting angular velocity");
  V3D angular_velocity_mean = V3D::Zero();
  ImuBuffer imu_measurements;
  try {
    imu_manager_->getInterpolatedMeasurements(
      corrected_ts_ - config_.frame_ms * 1e-3 / 2, corrected_ts_ + config_.frame_ms * 1e-3 / 2,
      imu_measurements);
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
    logger_->error(
      "Failed to get interpolated IMU measurements for angular velocity calculation. "
      "Skipping pointcloud processing.");
    return;
  }

  size_t number_of_measurements = 0;
  for (auto it = imu_measurements.begin(); it != imu_measurements.end(); ++it) {
    angular_velocity_mean += it->second.tail<3>();
    number_of_measurements++;
  }
  angular_velocity_mean /= (double)number_of_measurements;
  logger_->trace("Angular velocity: {}", angular_velocity_mean.transpose());

  if (valid_targets_.size() < 3) {
    logger_->warn("Few valid targets after filtering: {}", valid_targets_.size());
  }
  debug_msg_.n_points_valid = valid_targets_.size();

  // TODO: Zero velocity check

  gtsam::NonlinearFactorGraph new_factors;
  auto dhf = std::make_shared<DopplerHessianFactor>(
    valid_targets_, config_.base.T_B_S, angular_velocity_mean, X(0), V(0), B(0), config_.noise_sigma,
    config_.huber_threshold, config_.outlier_threshold);
  new_factors.add(dhf);

  logger_->debug("Declaring (ts: {})", corrected_ts_);
  Stopwatch sw_declare;
  graph::Manager::DeclarationResult dr =
    graph_manager_->declare(corrected_ts_, new_key_, config_.base.use_to_init, new_factors);

  if (!handleDeclarationResult(dr)) {
    return;
  }
  initialized_ = true;
  debug_msg_.t_graph_declare = sw_declare.elapsedMs();

  debug_msg_.n_points_static = dhf->getStatic().size();
  debug_msg_.n_points_dynamic = dhf->getDynamic().size();
  debug_msg_.header.stamp.fromSec(corrected_ts_);
  debug_msg_.t_full = sw.elapsedMs();
  logger_->debug(
    "Finished processing pointcloud in {} ms. Was assigned key: {}", debug_msg_.t_full,
    gdkf(new_key_));
  pub_debug_.publish(debug_msg_);
}

template <typename PointT>
void Manager::preprocess(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  Stopwatch sw;
  pcl::PointCloud<mmWavePoint> points;

  // Convert the cloud
  if constexpr (std::is_same<PointT, mmWavePoint>::value) {
    pcl::fromROSMsg(*msg, points);
  } else if constexpr (std::is_same<PointT, rioPoint>::value) {
    pcl::PointCloud<rioPoint> temp;
    pcl::fromROSMsg(*msg, temp);

    points.resize(temp.size());
    for (size_t i = 0; i < temp.size(); i++) {
      points[i].x = temp[i].y;
      points[i].y = -temp[i].x;
      points[i].z = temp[i].z;
      points[i].intensity = temp[i].snr_db;
      points[i].velocity = temp[i].v_doppler_mps;
    }
  } else {
    throw std::runtime_error("Unsupported point type");
  }
  debug_msg_.n_points_in = points.size();

  // Filter the pointcloud
  valid_targets_.clear();
  valid_targets_.reserve(points.size());
  pcl::PointCloud<mmWavePoint> valid_points;
  valid_points.reserve(points.size());
  for (const auto & point : points) {
    // NaN filter
    if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z) || std::isnan(point.intensity) || std::isnan(point.velocity))
    {
      logger_->trace("Point contains NaN values. Skipping this point.");
      continue;
    }

    if (point.intensity < config_.filter_min_db) {
      continue;
    }

    // calculate metrics for filtering
    const double range = point.getVector3fMap().norm();
    if (range < config_.range_min || range > config_.range_max) {
      continue;
    }

    const double azimuth = std::atan2(point.y, point.x);
    if (std::abs(azimuth) > deg2rad(config_.threshold_azimuth_deg)) {
      continue;
    }

    const double elevation = std::atan2(point.z, std::sqrt(point.x * point.x + point.y * point.y));
    if (std::abs(elevation) > deg2rad(config_.threshold_elevation_deg)) {
      continue;
    }

    valid_targets_.emplace_back(
      point.x, point.y, point.z, range, azimuth, elevation, point.velocity, point.intensity);
    valid_points.push_back(point);
  }

  // Publish the filtered pointcloud
  publishCloud(pub_filtered_points_, valid_points, config_.base.sensor_frame, corrected_ts_);

  logger_->debug("Valid targets: {}", valid_targets_.size());
  debug_msg_.t_preprocess = sw.elapsedMs();
}

void Manager::getFactors(gtsam::NonlinearFactorGraph & new_factors) {}

void declare_config(ManagerConfig & config)
{
  using namespace config;
  name("Radar Manager Config");

  // Declare common sensor manager config fields
  declare_sensor_manager_config_base(config.base, "radar");

  // Declare radar-specific fields
  {
    NameSpace ns("radar");
    {
      NameSpace ns("manager");
      field(config.is_exposure_compensated, "is_exposure_compensated", "bool");
      field(config.range_min, "range_min", "m");
      field(config.range_max, "range_max", "m");
      field(config.threshold_azimuth_deg, "threshold_azimuth_deg", "deg");
      field(config.threshold_elevation_deg, "threshold_elevation_deg", "deg");
      field(config.filter_min_db, "filter_min_db", "dB");
      field(config.frame_ms, "frame_ms", "float");
      field(config.noise_sigma, "noise_sigma", "float");
      field(config.huber_threshold, "huber_threshold", "float");
      field(config.outlier_threshold, "outlier_threshold", "float");
    }
  }
}

}  // namespace radar
}  // namespace mimosa
