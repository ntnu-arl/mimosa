// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/imu/manager.hpp"

namespace mimosa
{
namespace imu
{
Manager::Manager(ros::NodeHandle & pnh)
: config_(config::checkValid(config::fromRos<ManagerConfig>(pnh)))
{
  // Prepare logger
  logger_ =
    createLogger(config_.logs_directory + "imu_manager.log", "imu::Manager", config_.log_level);
  logger_->debug("imu::Manager initialized with params:\n {}", config::toString(config_));

  // Publishers
  pub_debug_ = pnh.advertise<mimosa_msgs::ImuManagerDebug>("imu/manager/debug", 1);
  pub_localizability_marker_array_ =
    pnh.advertise<visualization_msgs::MarkerArray>("imu/manager/localizability_marker_array", 1);
  pub_odometry_ = pnh.advertise<nav_msgs::Odometry>("imu/manager/odometry", 1);

  // Subscribe to IMU messages
  sub_ = pnh.subscribe(
    "imu/manager/imu_in", 1000, &Manager::callback, this, ros::TransportHints().tcpNoDelay());
}

std::shared_ptr<gtsam::PreintegrationParams> Manager::generatePreintegratorParams(
  const V3D & gravity) const
{
  const auto & c = config_.preintegration;

  auto p = std::make_shared<gtsam::PreintegrationParams>(gravity);

  // Accelerometer
  p->setAccelerometerCovariance(I3 * std::pow(c.acc_noise_density, 2));

  // The error made from integrating the position from the velocities
  p->setIntegrationCovariance(I3 * std::pow(c.integration_sigma, 2));

  // Gyroscope
  p->setGyroscopeCovariance(I3 * std::pow(c.gyro_noise_density, 2));

  // Other parameters
  p->setUse2ndOrderCoriolis(c.use_2nd_order_coriolis);

  return p;
}

std::shared_ptr<gtsam::PreintegrationParams> Manager::generatePreintegratorParams() const
{
  return generatePreintegratorParams(V3D(0, 0, -config_.preintegration.gravity_magnitude));
}

void Manager::callback(const sensor_msgs::Imu::ConstPtr & msg)
{
  const double ts = msg->header.stamp.toSec() + config_.ts_offset;
  // Check that the timestamp is strictly monotonic
  static double prev_ts = 0.0;
  if (ts <= prev_ts) {
    logger_->warn(
      "IMU message timestamp is not strictly monotonic. Current: {} Previous: {}, skipping this "
      "measurement",
      ts, prev_ts);
    return;
  }
  prev_ts = ts;

  if (!has_recieved_first_message_) {
    has_recieved_first_message_ = true;
  }

  V6D imu_meas;
  imu_meas << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
    msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

  if (config_.acc_scale_factor != 1.0) {
    // This scale factor is typically to account for the fact that the accelerometer values
    // are in units of g instead of m/s^2
    imu_meas.head<3>() *= config_.acc_scale_factor;
  }

  // Add to buffer
  ImuBuffer::const_iterator it_current, it_previous;
  bool propagate = false;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer_.emplace_back(ts, imu_meas);
    if (ts - buffer_.begin()->first > config_.max_buffer_duration) {
      // Remove oldest IMU measurement ensuring that the buffer time length is always the max duration
      buffer_.pop_front();
    }

    // Propagate the imu
    if (propagation_preintegrator_ && propagation_base_state_.ts() != 0) {
      it_current = std::prev(buffer_.end());
      it_previous = std::prev(it_current);
      propagate = true;
    }
  }

  if (propagate) {
    gtsam::NavState nav_state;
    {
      logger_->trace("Propagating IMU");
      std::lock_guard<std::mutex> lock(propagation_mutex_);
      if (propagated_upto_ts_ > it_current->first) {
        // The current imu measurement is older than the last propagated state
        logger_->warn(
          "Current IMU measurement is older than the last propagated state. Current ts: {} "
          "Propagated ts: {}",
          it_current->first, propagated_upto_ts_);
        return;
      }

      if (propagated_upto_ts_ < it_previous->first) {
        // Normally we will always be doing 1 step propagation, but if we have missed some imu
        // measurements, we need to propagate upto the current imu measurement
        updatePreintegrationTo(
          propagation_base_state_.ts(), it_previous->first, propagation_base_state_.imuBias(),
          propagation_preintegrator_);
        propagated_upto_ts_ = it_previous->first;
      }

      double dt = it_current->first - propagated_upto_ts_;

      // Update the preintegrator with the previous imu measurement for the dt
      propagation_preintegrator_->integrateMeasurement(
        it_previous->second.head<3>(), it_previous->second.tail<3>(), dt);
      propagated_upto_ts_ = it_current->first;

      // Predict the state
      nav_state = propagation_preintegrator_->predict(
        propagation_base_state_.navState(), propagation_base_state_.imuBias(),
        propagation_base_state_.gravity());
    }

    // Publish this new nav_state
    // Publish odometry
    nav_msgs::Odometry odometry;
    if (pub_odometry_.getNumSubscribers()) {
      odometry.header.frame_id = config_.map_frame;
      odometry.child_frame_id = config_.body_frame;
      odometry.header.stamp.fromSec(ts);
      convert(nav_state.pose(), odometry.pose.pose);
      convert(nav_state.velocity(), odometry.twist.twist.linear);
      pub_odometry_.publish(odometry);
    }
  }
}

bool Manager::estimateAttitude(
  gtsam::Rot3 & R_W_B, V3D & estimated_acc_bias, V3D & estimated_gyro_bias)
{
  size_t buffer_size = 0;
  V3D acc_mean(0.0, 0.0, 0.0), gyro_mean(0.0, 0.0, 0.0);
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    if (buffer_.empty()) {
      logger_->warn("IMU buffer is empty");
      return false;
    }
    const double buffer_duration = buffer_.rbegin()->first - buffer_.begin()->first;
    if (buffer_duration < config_.pose_init_wait_secs) {
      logger_->debug(
        "Imu buffer has not been filled enough yet. Current duration/Required duration: {} / {}",
        buffer_duration, config_.pose_init_wait_secs);
      return false;
    }

    // Accumulate all the measurements
    for (const auto & itr : buffer_) {
      acc_mean += itr.second.head<3>();
      gyro_mean += itr.second.tail<3>();
    }
    buffer_size = buffer_.size();
  }

  // Average IMU measurements
  acc_mean /= buffer_size;
  gyro_mean /= buffer_size;

  // Set the gyro bias - this assumes that we are stationary
  estimated_gyro_bias = gyro_mean;

  if (config_.preintegration.gravity_aligned_initialization) {
    const V3D gravity_W = V3D(0.0, 0.0, config_.preintegration.gravity_magnitude);

    // Assuming bias is small (magnitude is less than 1m/s^2) the max angle between
    // the true gravity in the body frame and the acc_mean is 0.0922 rad or 5.28 degrees
    gtsam::Rot3 R_I_W = gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(gravity_W, acc_mean));
    R_W_B = R_I_W.inverse();

    // This orientation is correct up to 5.28 degrees assuming magnitude(bias) < 1m/s^2

    // Assuming this orientation, the gravity vector is simply the acc_mean rescaled to the gravity magnitude
    const V3D estimated_gravity_vector =
      acc_mean.normalized() * config_.preintegration.gravity_magnitude;

    // This is then used to estimate the accelerometer bias
    estimated_acc_bias = acc_mean - estimated_gravity_vector;

    preintegrator_params_ = generatePreintegratorParams();
  } else {
    // Initial orientation is identity
    R_W_B = gtsam::Rot3::Identity();
    // Gravity is the direction that allows this to be identity
    V3D gravity = -acc_mean.normalized() *
                  config_.preintegration.gravity_magnitude;
    estimated_acc_bias = acc_mean + gravity;
    preintegrator_params_ = generatePreintegratorParams(gravity);
  }
  preintegrator_ = std::make_unique<gtsam::PreintegratedImuMeasurements>(preintegrator_params_);
  propagation_preintegrator_ =
    std::make_unique<gtsam::PreintegratedImuMeasurements>(preintegrator_params_);

  logger_->info("Estimated R_W_B:\n{}", R_W_B.matrix());
  logger_->info("Estimated acc bias: {}", estimated_acc_bias.transpose());
  logger_->info("Estimated gyro bias: {}", estimated_gyro_bias.transpose());
  logger_->info(
    "Gravity vector: {} with norm: {}", preintegrator_params_->getGravity().transpose(),
    preintegrator_params_->getGravity().norm());

  return true;
}

V6D Manager::interpolateMeasurement(
  const double ts1, const V6D & meas1, const double ts2, const V6D & meas2, const double ts)
{
  if (ts1 == ts2) {
    logCriticalException<std::invalid_argument>(
      logger_, fmt::format("Cannot interpolate between two identical timestamps: {}", ts1));
  }

  if (ts < ts1) {
    logCriticalException<std::invalid_argument>(
      logger_, fmt::format(
                 "IMU interpolation for timestamp that is before ts1. ts1: {}, requested ts: {}, "
                 "difference: {}",
                 ts1, ts, ts - ts1));
  }

  if (ts > ts2 && ts - ts2 > config_.extrapolation_max_ts_diff) {
    logCriticalException<std::invalid_argument>(
      logger_,
      fmt::format(
        "IMU extrapolation for timestamp that is too far past ts2. ts_diff = {} Max allowed: {}",
        ts - ts2, config_.extrapolation_max_ts_diff));
  }

  if (ts > ts1 && ts < ts2 && ts2 - ts1 > config_.interpolation_max_ts_diff) {
    logCriticalException<std::invalid_argument>(
      logger_, fmt::format(
                 "IMU interpolation between timestamps that are too far apart: {}/{} diff: {} Max "
                 "allowed: {}",
                 ts1, ts2, ts2 - ts1, config_.interpolation_max_ts_diff));
  }

  // Now the actual interpolation
  const double ratio = (ts - ts1) / (ts2 - ts1);
  return meas1 + (meas2 - meas1) * ratio;
}

void Manager::getInterpolatedMeasurements(
  const double ts_start, const double ts_end, ImuBuffer & measurements,
  const bool dont_interpolate_first_measurement)
{
  // Check if timestamps are in correct order
  if (ts_start >= ts_end) {
    logCriticalException<std::invalid_argument>(
      logger_, fmt::format("IMU lookup timestamps ts_start({}) >= ts_end({})", ts_start, ts_end));
  }

  measurements.clear();

  std::lock_guard<std::mutex> lock(buffer_mutex_);

  auto s_itr = std::lower_bound(
    buffer_.begin(), buffer_.end(), ts_start,
    [](const ImuMeasurement & m, const double ts) { return m.first < ts; });

  auto e_itr = std::lower_bound(
    buffer_.begin(), buffer_.end(), ts_end,
    [](const ImuMeasurement & m, const double ts) { return m.first < ts; });

  // Check if it is first value in the buffer which means there is no value before to interpolate with
  if (s_itr == buffer_.begin()) {
    logCriticalException<std::runtime_error>(
      logger_,
      fmt::format(
        "IMU lookup requiring first message of the buffer s_itr == buffer_.begin() ts_start: {}, "
        "s_itr->first: {} . This should never happen. Check the ts that you are querying.",
        ts_start, s_itr->first));
  }

  if (s_itr == buffer_.end()) {
    logCriticalException<std::runtime_error>(
      logger_,
      fmt::format(
        "IMU lookup past the end of the buffer s_itr == buffer_.end() ts_start: {}, ts of last "
        "message in buffer : {} . This should never happen. Check the ts that you are querying.",
        ts_start, buffer_.rbegin()->first));
  }

  // Check if two IMU messages are different
  if (s_itr == e_itr) {
    logger_->trace(
      "Not Enough IMU measurements between timestamps, with Start/End Timestamps: {}/{} The only "
      "measurement is at: {}",
      ts_start, ts_end, s_itr->first);
  } else {
    // Copy in between IMU measurements in [s_itr, e_itr)
    for (auto itr = s_itr; itr != e_itr; ++itr) {
      measurements.push_back(*itr);
    }
  }

  if (s_itr->first > ts_start) {
    // Since s_itr is found with lower_bound, it can only be greater than or equal to
    // ts_start. If it is equal, then the exact value has already been added to the measurements

    // Interpolate first element
    auto prev_s_itr = s_itr;
    --prev_s_itr;
    if (dont_interpolate_first_measurement) {
      measurements.emplace_front(ts_start, prev_s_itr->second);
    } else {
      measurements.emplace_front(
        ts_start, interpolateMeasurement(
                    prev_s_itr->first, prev_s_itr->second, s_itr->first, s_itr->second, ts_start));
    }
  }

  // Add the last element
  if (e_itr == buffer_.end()) --e_itr;  // Make the iterator valid for interpolation

  // This is not scoped since the last element is not inserted into the map -> Hence this is always required
  auto prev_e_itr = e_itr;
  --prev_e_itr;
  measurements.emplace_back(
    ts_end, interpolateMeasurement(
              prev_e_itr->first, prev_e_itr->second, e_itr->first, e_itr->second, ts_end));

  // If e_itr happens to have the same time as ts_end, then this will just do an extra interpolation,
  // but that is ok since this is extremely cheap and almost never happens.
}

size_t Manager::getNumMeasurementsBetween(const double t1, const double t2)
{
  const double t_start = std::min(t1, t2);
  const double t_end = std::max(t1, t2);

  std::lock_guard<std::mutex> lock(buffer_mutex_);

  auto s_itr = std::lower_bound(
    buffer_.begin(), buffer_.end(), t_start,
    [](const ImuMeasurement & m, const double ts) { return m.first < ts; });

  size_t count = 0;
  for (auto it = s_itr; it != buffer_.end(); ++it) {
    if (it->first > t_end) {
      break;
    }
    ++count;
  }

  return count;
}


void Manager::updatePreintegrationTo(
  const double ts_start, const double ts_end, const gtsam::imuBias::ConstantBias & bias,
  std::unique_ptr<gtsam::PreintegratedImuMeasurements> & preintegrator)
{
  // Preintegrate the imu measurements
  ImuBuffer measurements;
  getInterpolatedMeasurements(ts_start, ts_end, measurements, true);

  if (measurements.size() < 2) {
    std::string msg = "Preintegration not possible as there are less than 2 measurements P1";
    logger_->error(msg);
    throw std::runtime_error(msg);
  }

  preintegrator->resetIntegrationAndSetBias(bias);

  // Integrate the measurements
  auto curr_itr = measurements.begin();
  auto next_itr = curr_itr;
  ++next_itr;
  for (; next_itr != measurements.end(); ++curr_itr, ++next_itr) {
    double dt = next_itr->first - curr_itr->first;
    preintegrator->integrateMeasurement(curr_itr->second.head<3>(), curr_itr->second.tail<3>(), dt);
  }
}

void Manager::addImuFactorNoLock(
  const double ts_0, const double ts_1, const gtsam::imuBias::ConstantBias & bias_0,
  const gtsam::Key key_0, const gtsam::Key key_1, gtsam::NonlinearFactorGraph & graph)
{
  updatePreintegrationTo(ts_0, ts_1, bias_0, preintegrator_);
  graph.add(gtsam::ImuFactorWithGravity(
    X(key_0), V(key_0), X(key_1), V(key_1), B(key_0), G(0), *preintegrator_));

  const auto imu_bias_random_walk =
    (V6D() << config_.preintegration.acc_bias_random_walk,
     config_.preintegration.acc_bias_random_walk, config_.preintegration.acc_bias_random_walk,
     config_.preintegration.gyro_bias_random_walk, config_.preintegration.gyro_bias_random_walk,
     config_.preintegration.gyro_bias_random_walk)
      .finished();
  graph.add(
    gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
      B(key_0), B(key_1), gtsam::imuBias::ConstantBias(V3D::Zero(), V3D::Zero()),
      gtsam::noiseModel::Diagonal::Sigmas(imu_bias_random_walk * sqrt(preintegrator_->deltaTij()))));

  logger_->debug("Added IMU factor between keys {} and {}", gdkf(key_0), gdkf(key_1));
}

void Manager::addImuFactor(
  const double ts_0, const double ts_1, const gtsam::imuBias::ConstantBias & bias_0,
  const gtsam::Key key_0, const gtsam::Key key_1, gtsam::NonlinearFactorGraph & graph)
{
  std::lock_guard<std::mutex> lock(preintegrator_mutex_);
  addImuFactorNoLock(ts_0, ts_1, bias_0, key_0, key_1, graph);
}

void Manager::addImuFactorAndGetNavState(
  const State & state_0, const double ts_1, const gtsam::Key key_1,
  gtsam::NonlinearFactorGraph & graph, gtsam::NavState & ns_1)
{
  {
    std::lock_guard<std::mutex> lock(preintegrator_mutex_);
    addImuFactorNoLock(state_0.ts(), ts_1, state_0.imuBias(), state_0.key(), key_1, graph);
    ns_1 = preintegrator_->predict(state_0.navState(), state_0.imuBias(), state_0.gravity());
    logger_->trace("predicted state");
  }

  gtsam::Values values;
  values.insert(X(state_0.key()), state_0.navState().pose());
  values.insert(V(state_0.key()), state_0.navState().velocity());
  values.insert(B(state_0.key()), state_0.imuBias());
  values.insert(X(key_1), ns_1.pose());
  values.insert(V(key_1), ns_1.velocity());
  values.insert(B(key_1), state_0.imuBias());
  values.insert(G(0), state_0.gravity());

  // The imu factor is the second to last factor in the graph
  auto & imu_factor = *graph.end()[-2];
  auto linearized_factor = imu_factor.linearize(values);
  auto & hessian = linearized_factor->hessianBlockDiagonal()[X(key_1)];

  M33 localizability_eigenvectors_trans, localizability_eigenvectors_rot;
  V3D localizability_trans, localizability_rot;
  computeLocalizability(
    hessian.topLeftCorner(3, 3), localizability_rot, localizability_eigenvectors_rot);
  computeLocalizability(
    hessian.bottomRightCorner(3, 3), localizability_trans, localizability_eigenvectors_trans);

  convert(localizability_rot, debug_msg_.imu_localizability_rot);
  convert(localizability_trans, debug_msg_.imu_localizability_trans);

  debug_msg_.header.stamp.fromSec(ts_1);
  pub_debug_.publish(debug_msg_);

  // Create the marker for the axis
  visualization_msgs::MarkerArray ma;
  addTriadMarker(
    localizability_eigenvectors_trans, config_.body_frame, ts_1, "LocalizabilityTrans", ma);
  addTriadMarker(
    localizability_eigenvectors_rot, config_.body_frame, ts_1, "LocalizabilityRot", ma);

  pub_localizability_marker_array_.publish(ma);
}

void Manager::setPropagationBaseState(const State & state)
{
  logger_->trace("Setting propagation base state");

  // we should propagate it to the latest IMU message in the buffer
  double latest_ts;
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    latest_ts = buffer_.rbegin()->first;
  }

  std::lock_guard<std::mutex> lock(propagation_mutex_);
  if (state.ts() > propagation_base_state_.ts()) {
    propagation_base_state_ = state;
    propagation_preintegrator_->resetIntegrationAndSetBias(propagation_base_state_.imuBias());
    propagated_upto_ts_ = propagation_base_state_.ts();
  }

  if (latest_ts > propagation_base_state_.ts()) {
    updatePreintegrationTo(
      propagation_base_state_.ts(), latest_ts, state.imuBias(), propagation_preintegrator_);
    propagated_upto_ts_ = latest_ts;
  }
}

void declare_config(PreintegrationConfig & config)
{
  using namespace config;
  name("Preintegration Config");
  field(config.acc_noise_density, "acc_noise_density", "m/s^2/sqrt(Hz)");
  field(config.acc_bias_random_walk, "acc_bias_random_walk", "m/s^3/sqrt(Hz)");
  field(config.gyro_noise_density, "gyro_noise_density", "rad/s/sqrt(Hz)");
  field(config.gyro_bias_random_walk, "gyro_bias_random_walk", "rad/s^2/sqrt(Hz)");
  field(config.integration_sigma, "integration_sigma", "-");
  field(config.use_2nd_order_coriolis, "use_2nd_order_coriolis", "bool");
  field(config.use_estimated_gravity, "use_estimated_gravity", "bool");
  field(config.gravity_magnitude, "gravity_magnitude", "m/s^2");
  field(config.gravity_aligned_initialization, "gravity_aligned_initialization", "bool");

  check(config.acc_noise_density, GT, 0.0, "acc_noise_density");
  check(config.acc_bias_random_walk, GT, 0.0, "acc_bias_random_walk");
  check(config.gyro_noise_density, GT, 0.0, "gyro_noise_density");
  check(config.gyro_bias_random_walk, GT, 0.0, "gyro_bias_random_walk");
  check(config.integration_sigma, GE, 0.0, "integration_sigma");
  check(config.gravity_magnitude, GT, 0.0, "gravity_magnitude");
}

void declare_config(ManagerConfig & config)
{
  using namespace config;
  name("Imu Manager Config");

  field(config.logs_directory, "logs_directory", "directory_path");
  field(config.map_frame, "map_frame", "str");
  field(config.body_frame, "body_frame", "str");

  {
    NameSpace ns("imu");
    {
      NameSpace ns("manager");
      field(config.log_level, "log_level", "trace|debug|info|warn|error|critical");
      field(config.ts_offset, "ts_offset", "s");
      field(config.max_buffer_duration, "max_buffer_duration", "s");
      field(config.pose_init_wait_secs, "pose_init_wait_secs", "s");
      field(config.interpolation_max_ts_diff, "interpolation_max_ts_diff", "s");
      field(config.extrapolation_max_ts_diff, "extrapolation_max_ts_diff", "s");
      field(config.acc_scale_factor, "acc_scale_factor");
    }
    field(config.preintegration, "preintegration");
  }
  check(config.max_buffer_duration, GT, 0.5, "max_buffer_duration");
  check(config.pose_init_wait_secs, GE, 0.0, "pose_init_wait_secs");
  check(config.interpolation_max_ts_diff, GT, 0.005, "interpolation_max_ts_diff");
  check(config.extrapolation_max_ts_diff, GT, 0.005, "extrapolation_max_ts_diff");
  check(config.acc_scale_factor, GE, 1.0, "acc_scale_factor");
  check(config.acc_scale_factor, LT, 10.0, "acc_scale_factor");
}

}  // namespace imu
}  // namespace mimosa
