// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/graph/manager.hpp"

namespace mimosa
{
namespace graph
{
Manager::Manager(ros::NodeHandle & pnh, mimosa::imu::Manager::Ptr imu_manager)
: config_(config::checkValid(config::fromRos<ManagerConfig>(pnh))),
  imu_manager_(imu_manager),
  isam2_params_(getISAM2Params())
{
  // Prepare logger
  logger_ =
    createLogger(config_.logs_directory + "graph_manager.log", "graph::Manager", config_.log_level);
  logger_->info("graph::Manager initialized with params:\n {}", config::toString(config_));

#if SMOOTHER_IFL
  smoother_ =
    std::make_unique<gtsam::IncrementalFixedLagSmoother>(config_.smoother.lag, isam2_params_);
#else
  smoother_ = std::make_unique<gtsam::ISAM2>(isam2_params_);
#endif

  initialized_ = false;

  pub_debug_ = pnh.advertise<mimosa_msgs::GraphManagerDebug>("graph/debug", 1);
  pub_path_ = pnh.advertise<nav_msgs::Path>("graph/path", 1);
  pub_odometry_ = pnh.advertise<nav_msgs::Odometry>("graph/odometry", 1);
  pub_transform_stamped_ = pnh.advertise<geometry_msgs::TransformStamped>("graph/transform", 1);
  pub_optimized_path_ = pnh.advertise<nav_msgs::Path>("graph/optimized_path", 1);
}

gtsam::ISAM2Params Manager::getISAM2Params() const
{
  gtsam::ISAM2GaussNewtonParams gnp;
  gnp.setWildfireThreshold(config_.smoother.wildfire_threshold);
  gtsam::ISAM2Params isam2_params(gnp);

  gtsam::FastMap<char, gtsam::Vector> relinearize_threshold;
  relinearize_threshold['x'] = (gtsam::Vector(6) << config_.smoother.relinearize_threshold_rotation,
                                config_.smoother.relinearize_threshold_rotation,
                                config_.smoother.relinearize_threshold_rotation,
                                config_.smoother.relinearize_threshold_translation,
                                config_.smoother.relinearize_threshold_translation,
                                config_.smoother.relinearize_threshold_translation)
                                 .finished();
  relinearize_threshold['v'] = (gtsam::Vector(3) << config_.smoother.relinearize_threshold_velocity,
                                config_.smoother.relinearize_threshold_velocity,
                                config_.smoother.relinearize_threshold_velocity)
                                 .finished();
  relinearize_threshold['b'] = (gtsam::Vector(6) << config_.smoother.relinearize_threshold_bias_acc,
                                config_.smoother.relinearize_threshold_bias_acc,
                                config_.smoother.relinearize_threshold_bias_acc,
                                config_.smoother.relinearize_threshold_bias_gyro,
                                config_.smoother.relinearize_threshold_bias_gyro,
                                config_.smoother.relinearize_threshold_bias_gyro)
                                 .finished();
  relinearize_threshold['g'] = (gtsam::Vector(2) << config_.smoother.relinearize_threshold_gravity,
                                config_.smoother.relinearize_threshold_gravity)
                                 .finished();
  isam2_params.setRelinearizeThreshold(relinearize_threshold);
  isam2_params.relinearizeSkip = config_.smoother.relinearize_skip;
  isam2_params.enableRelinearization = config_.smoother.enable_relinearization;
  isam2_params.evaluateNonlinearError = config_.smoother.evaluate_nonlinear_error;
  if (config_.smoother.factorization == "QR") {
    isam2_params.factorization = gtsam::ISAM2Params::QR;
  } else if (config_.smoother.factorization == "CHOLESKY") {
    isam2_params.factorization = gtsam::ISAM2Params::CHOLESKY;
  } else {
    throw std::runtime_error("Unsupported factorization type");
  }
  isam2_params.cacheLinearizedFactors = config_.smoother.cache_linearized_factors;
  isam2_params.enableDetailedResults = config_.smoother.enable_detailed_results;
  isam2_params.enablePartialRelinearizationCheck =
    config_.smoother.enable_partial_relinearization_check;

  isam2_params.findUnusedFactorSlots =
    SMOOTHER_IFL;  // Necessary for fixed lag smoothing hence this is not a parameter

  return isam2_params;
}

Manager::DeclarationResult Manager::declare(
  const double ts, gtsam::Key & key, const bool use_to_init)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);

  key = getNextKey();

  logger_->info("Declaring key: {}", gtsam::DefaultKeyFormatter(key));

  if (!initialized_) {
    if (!use_to_init) {
      return DeclarationResult::FAILURE_CANNOT_INIT_ON_MODALITY;
    }
    // Try to initialize
    gtsam::Rot3 R_W_B;
    V3D estimated_acc_bias, estimated_gyro_bias;
    if (!imu_manager_->estimateAttitude(R_W_B, estimated_acc_bias, estimated_gyro_bias)) {
      logger_->warn("Failed to estimate attitude");
      resetKey();
      return DeclarationResult::FAILURE_ATTITUDE_ESTIMATION;
    }

    logger_->info("Estimated R_W_B:\n{}", R_W_B.matrix());
    logger_->info("Estimated acc bias: {}", estimated_acc_bias.transpose());
    logger_->info("Estimated gyro bias: {}", estimated_gyro_bias.transpose());

    const auto imu_bias = gtsam::imuBias::ConstantBias(estimated_acc_bias, estimated_gyro_bias);
    const gtsam::Pose3 T_W_Bk = gtsam::Pose3(R_W_B, V3D::Zero());

    initializeGraph(ts, key, T_W_Bk, V3D::Zero(), imu_bias);
    initialized_ = true;
    return DeclarationResult::SUCCESS_INITIALIZED;
  }

  // If the timestamp difference is less than the sampling time of the imu, then no need to make a new state
  const double ts_diff = ts - state_.ts();
  if (std::abs(ts_diff) < config_.max_ts_diff_for_imu_breaking) {
    key = state_.key();
    logger_->info("Using same key: {}", gtsam::DefaultKeyFormatter(key));
    logger_->trace("unlocked in declare");
    return DeclarationResult::SUCCESS_SAME_KEY;
  }

  if (ts < state_.ts()) {
    Stopwatch sw;
    // Out of order measurement
    logger_->debug("Handling out of order with diff: {}", ts_diff);

#if SMOOTHER_IFL
    // If the timestamp is outside the smoother lag window, then it cannot be handled
    if (ts < smoother_->timestamps().begin()->second) {
      logger_->warn("Timestamp is older than entire lag window. Ignoring measurement");
      return DeclarationResult::FAILURE_OLDER_THAN_LAG;
    }
#endif

    // If the timestamp is too old, then there is no point in handling it
    // The idea is that measurements with high latency should never happen in the first place
    if (std::abs(ts_diff) > config_.max_measurement_latency) {
      logger_->warn(
        "Timestamp is too old to be considered with ts_diff: {}. Ignoring measurement", ts_diff);
      return DeclarationResult::FAILURE_OLDER_THAN_MAX_LATENCY;
    }

    if (ts_key_map_.size() == 1) {
      // The only state is the current state - i.e. the new measurement is pre intialization
      // The measurement that caused this can then be ignored
      logger_->debug("Ignoring out of order measurement");
      return DeclarationResult::FAILURE_OLDER_THAN_INITIALIZATION;
    }

    // Get the state that precedes the timestamp
    State prev_state;
    getStateUptoNoLock(ts, prev_state);

    gtsam::NonlinearFactorGraph new_factors;

    // Get the propagated state and IMU factor
    gtsam::NavState propagated_state;
    imu_manager_->addImuFactorAndGetNavState(prev_state, ts, key, new_factors, propagated_state);
    auto & imu_factor_1 = *new_factors.end()[-2];

    logger_->debug("Got prev_state (ts: {})", prev_state.ts());

    // Add the new state to the smoother
    gtsam::Values values;
    values.insert(X(key), propagated_state.pose());
    values.insert(V(key), propagated_state.velocity());
    values.insert(B(key), prev_state.imuBias());

    gtsam::IncrementalFixedLagSmoother::KeyTimestampMap key_ts_map;
    for (const auto & [k, value] : values) {
      key_ts_map[k] = ts;
    }
    ts_key_map_[ts] = key;  // This is the key without the character

    imu_manager_->addImuFactor(
      ts, state_.ts(), prev_state.imuBias(), key, state_.key(), new_factors);
    auto & imu_factor_2 = *new_factors.end()[-2];

    logger_->debug("Got middle state (ts: {})", ts);
    logger_->debug("Got end state (ts: {})", state_.ts());

    // Find the imu factor that connects the previous state to the current state

    int index = -1;
#if SMOOTHER_IFL
    const auto factors_in_graph = smoother_->getFactors();
    // Iterate forwards in the factors to find the imu factor that connects the previous state to the current state
    for (int i = 0; i < factors_in_graph.size(); ++i) {
      logger_->debug("checking: {}", i);
      try {
        if (!factors_in_graph[i]) continue;

        const auto f = factors_in_graph[i];
        logger_->debug("f: {}", f->keys().size());
        if (
          f->dim() == imu_factor_1.dim() && f->keys().size() == imu_factor_1.keys().size() &&
          f->keys()[0] == imu_factor_1.keys()[0] && f->keys()[2] == imu_factor_2.keys()[2]) {
          index = i;
          break;
        }
      } catch (const std::exception & e) {
        std::cerr << e.what() << '\n';
        throw e;
      }
    }
#else
    const auto factors_in_graph = smoother_->getFactorsUnsafe();
    // Iterate backwards in the factors to find the imu factor that connects the previous state to the current state
    for (int i = factors_in_graph.size() - 1; i >= 0; --i) {
      logger_->debug("checking: {}", i);
      try {
        if (!factors_in_graph[i]) continue;

        const auto f = factors_in_graph[i];
        logger_->debug("f: {}", f->keys().size());
        if (
          f->dim() == imu_factor_1.dim() && f->keys().size() == imu_factor_1.keys().size() &&
          f->keys()[0] == imu_factor_1.keys()[0] && f->keys()[2] == imu_factor_2.keys()[2]) {
          index = i;
          break;
        }
      } catch (const std::exception & e) {
        std::cerr << e.what() << '\n';
        throw e;
      }
    }
#endif

    if (index == -1) {
      logCriticalException<std::runtime_error>(
        logger_,
        "Could not find the imu factor that connects the previous state to the current state");
    }
    logger_->debug("Removing factor at index: {}", index);

    gtsam::FactorIndices factors_to_remove;
    factors_to_remove.push_back(index);

#if SMOOTHER_IFL
    smoother_->update(new_factors, values, key_ts_map, factors_to_remove);
#else
    smoother_->update(new_factors, values, factors_to_remove);
#endif

    // optimized_values_ = smoother_->calculateEstimate();

    // Update the current state
    updateStateToKeyTs(state_.key(), state_.ts());

    logger_->debug("Handled out of order measurement in {} ms", sw.elapsedMs());
    return DeclarationResult::SUCCESS_OUT_OF_ORDER;
  }

  // Create a new state at ts
  // Add a preintegrated imu factor between the current state and this new state
  // Add the new state to the smoother
  // Optimize

  // TODO: From this point on, the rest of this function does not need to keep the caller waiting. However, the rest of the code must hold on to the mutex

  logger_->info("Adding imu factor");

  // Get the propagated state and imu factor
  gtsam::NavState propagated_state;
  gtsam::NonlinearFactorGraph new_factors;
  imu_manager_->addImuFactorAndGetNavState(state_, ts, key, new_factors, propagated_state);

  logger_->info("Added imu factor");
  // Add the new state to the smoother
  gtsam::Values values;
  values.insert(X(key), propagated_state.pose());
  values.insert(V(key), propagated_state.velocity());
  values.insert(B(key), state_.imuBias());

  logger_->info("Running update");
#if SMOOTHER_IFL
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap key_ts_map;
  for (const auto & [k, value] : values) {
    key_ts_map[k] = ts;
  }
  key_ts_map[G(0)] = ts;
  smoother_->update(new_factors, values, key_ts_map);
#else
  smoother_->update(new_factors, values);
#endif

  ts_key_map_[ts] = key;  // This is the key without the character
  optimized_values_ = smoother_->calculateEstimate();

  // Update the current state
  updateStateToKeyTs(key, ts);

  return Manager::DeclarationResult::SUCCESS_NORMAL;
}

void Manager::getCurrentState(State & state)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  state = state_;
}

void Manager::getStateUpto(const double ts, State & state)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  logger_->trace("Locked in getStateUpto");
  getStateUptoNoLock(ts, state);
  logger_->trace("unlocked in getStateUpto");
}

void Manager::getStateUptoNoLock(const double ts, State & state)
{
  if (ts_key_map_.empty()) {
    // This means that there are no states added yet. This only happens before initialization
    return;
  }

  auto it = ts_key_map_.upper_bound(ts);
  if (it == ts_key_map_.begin()) {
    // This means that the timestamp is older than all the states that have ever been added
    logCriticalException<std::runtime_error>(
      logger_, fmt::format("No state available upto ts: {}", ts));
  }

  --it;
  const auto key = it->second;

  state.update(
    key, it->first,
    gtsam::NavState(
      smoother_->calculateEstimate<gtsam::Pose3>(X(key)),
      smoother_->calculateEstimate<V3D>(V(key))),
    smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(key)),
    smoother_->calculateEstimate<gtsam::Unit3>(G(0)));
}

void Manager::define(
  const gtsam::NonlinearFactorGraph & graph, gtsam::Values & optimized_values,
  const DeclarationResult result)
{
  // All the factors in the graph are attached to variables that already exist in the smoother
  // Hence, we can just add the graph to the smoother
  std::lock_guard<std::mutex> lock(graph_mutex_);
  smoother_->update(graph);
  for (int i = 0; i < config_.smoother.additional_update_iterations; ++i) {
    smoother_->update();
  }

  optimized_values_ = smoother_->calculateEstimate();

  optimized_values = optimized_values_;

  // Update the current state
  updateStateToKeyTs(state_.key(), state_.ts());

  imu_manager_->setPropagationBaseState(state_);

  if (
    result == DeclarationResult::SUCCESS_SAME_KEY ||
    result == DeclarationResult::SUCCESS_OUT_OF_ORDER) {
    logger_->debug("Not publishing results as the key is the same");
  } else {
    publishResults();
  }
}

void Manager::updateStateToKeyTs(const gtsam::Key key, const double ts)
{
  // Update the state to be at key
  const auto p = smoother_->calculateEstimate<gtsam::Pose3>(X(key));
  const auto v = smoother_->calculateEstimate<V3D>(V(key));
  const auto b = smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(key));
  const auto g = smoother_->calculateEstimate<gtsam::Unit3>(G(0));

  logger_->info("Updated gravity is: {}", g.unitVector().transpose());

  const auto pose_diff = p.between(state_.navState().pose());
  debug_msg_.diff_against_imu_prior_trans_cm = pose_diff.translation().norm() * 100;
  debug_msg_.diff_against_imu_prior_rot_deg = rad2deg(pose_diff.rotation().axisAngle().second);

  state_.update(key, ts, gtsam::NavState(p, v), b, g);
}

void Manager::initializeGraph(
  const double ts, const gtsam::Key key, const gtsam::Pose3 & T_W_B, const V3D & vel,
  const gtsam::imuBias::ConstantBias & bias)
{
  const auto prior_noise_pose = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << deg2rad(config_.smoother.initial_rotation_pitch_roll_sigma_deg),
     deg2rad(config_.smoother.initial_rotation_pitch_roll_sigma_deg),
     deg2rad(config_.smoother.initial_rotation_yaw_sigma_deg),
     config_.smoother.initial_position_sigma, config_.smoother.initial_position_sigma,
     config_.smoother.initial_position_sigma)
      .finished());  // rad, rad, rad, m, m, m
  const auto prior_noise_velocity =
    gtsam::noiseModel::Isotropic::Sigma(3, config_.smoother.initial_velocity_sigma);  // m/s
  const auto prior_noise_bias = gtsam::noiseModel::Diagonal::Sigmas(
    (gtsam::Vector(6) << config_.smoother.initial_bias_acc_sigma,
     config_.smoother.initial_bias_acc_sigma, config_.smoother.initial_bias_acc_sigma,
     config_.smoother.initial_bias_gyro_sigma, config_.smoother.initial_bias_gyro_sigma,
     config_.smoother.initial_bias_gyro_sigma)
      .finished());  // acc, acc, acc, gyro, gyro, gyro
  const auto prior_noise_gravity =
    gtsam::noiseModel::Isotropic::Sigma(2, config_.smoother.initial_gravity_sigma);  // m/s^2

  const gtsam::Unit3 n_gravity_direction =
    gtsam::Unit3(imu_manager_->getPreintegratorParams()->getGravity());

  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(key), T_W_B, prior_noise_pose);
  graph.emplace_shared<gtsam::PriorFactor<V3D>>(V(key), vel, prior_noise_velocity);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
    B(key), bias, prior_noise_bias);
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Unit3>>(
    G(0), n_gravity_direction, prior_noise_gravity);

  gtsam::Values initial_values;
  initial_values.insert(X(key), T_W_B);
  initial_values.insert(V(key), vel);
  initial_values.insert(B(key), bias);
  initial_values.insert(G(0), n_gravity_direction);

#if SMOOTHER_IFL
  gtsam::IncrementalFixedLagSmoother::KeyTimestampMap key_ts_map;
  for (const auto & [key, value] : initial_values) {
    key_ts_map[key] = ts;
  }
  smoother_->update(graph, initial_values, key_ts_map);
#else
  smoother_->update(graph, initial_values);
#endif
  optimized_values_ = smoother_->calculateEstimate();

  ts_key_map_[ts] = key;

  updateStateToKeyTs(key, ts);

  publishResults();
}

void Manager::publishResults()
{
  broadcastTransform(
    tf2_broadcaster_, state_.navState().pose(), config_.world_frame, config_.body_frame,
    state_.ts());

  // Broadcast the nav to world transform
  const V3D n_g_direction = -V3D::UnitZ();
  V3D w_g_direction = state_.gravity().unitVector();
  // n_g = T_N_W * w_g
  gtsam::Pose3 T_N_W = gtsam::Pose3(
    gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(w_g_direction, n_g_direction)), V3D::Zero());
  broadcastTransform(
    tf2_static_broadcaster_, T_N_W, config_.navigation_frame, config_.world_frame, 0.0);

  static nav_msgs::Path path;
  path.header.frame_id = config_.world_frame;
  publishPath(pub_path_, path, state_.navState().pose(), state_.ts());

  // Publish odometry
  static nav_msgs::Odometry odometry;
  if (pub_odometry_.getNumSubscribers()) {
    odometry.header.frame_id = config_.world_frame;
    odometry.child_frame_id = config_.body_frame;
    odometry.header.stamp.fromSec(state_.ts());
    convert(state_.navState().pose(), odometry.pose.pose);
    convert(state_.navState().velocity(), odometry.twist.twist.linear);
    pub_odometry_.publish(odometry);
  }

  // Publish transform stamped
  static geometry_msgs::TransformStamped ts_frame_child;
  if (pub_transform_stamped_.getNumSubscribers()) {
    ts_frame_child.header.frame_id = config_.world_frame;
    ts_frame_child.child_frame_id = config_.body_frame;
    ts_frame_child.header.stamp.fromSec(state_.ts());
    convert(state_.navState().pose(), ts_frame_child.transform);
    pub_transform_stamped_.publish(ts_frame_child);
  }

  // Publish debug message
  debug_msg_.header.stamp.fromSec(state_.ts());
  convert(state_.imuBias().accelerometer(), debug_msg_.acc_bias);
  convert(state_.imuBias().gyroscope(), debug_msg_.gyro_bias);
  convert(
    state_.gravity().unitVector() * imu_manager_->config().preintegration.gravity_magnitude,
    debug_msg_.gravity);
  pub_debug_.publish(debug_msg_);

  // Iterate over the optimized values and publish the poses
  nav_msgs::Path opt_path;
  opt_path.header.frame_id = config_.world_frame;
  for (const auto & [key, value] : optimized_values_) {
    // Check if key is of pose type
    if (*gtsam::DefaultKeyFormatter(key).begin() != 'x') {
      continue;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = config_.world_frame;
    pose.header.stamp.fromSec(state_.ts());
    convert(value.cast<gtsam::Pose3>(), pose.pose);
    opt_path.poses.push_back(pose);
  }

  pub_optimized_path_.publish(opt_path);
}

void declare_config(SmootherConfig & config)
{
  using namespace config;
  name("Smoother Config");

  field(config.lag, "lag", "s");
  field(config.wildfire_threshold, "wildfire_threshold", "wildfire threshold");
  field(config.relinearize_threshold_translation, "relinearize_threshold_translation", "m");
  field(config.relinearize_threshold_rotation, "relinearize_threshold_rotation", "rad");
  field(config.relinearize_threshold_velocity, "relinearize_threshold_velocity", "m/s");
  field(config.relinearize_threshold_bias_acc, "relinearize_threshold_bias_acc", "m/s^2");
  field(config.relinearize_threshold_bias_gyro, "relinearize_threshold_bias_gyro", "rad/s");
  field(config.relinearize_threshold_gravity, "relinearize_threshold_gravity", "m/s^2");
  field(config.relinearize_skip, "relinearize_skip", "num");
  field(config.enable_relinearization, "enable_relinearization", "bool");
  field(config.evaluate_nonlinear_error, "evaluate_nonlinear_error", "bool");
  field(config.factorization, "factorization", "QR|CHOLESKY");
  field(config.cache_linearized_factors, "cache_linearized_factors", "bool");
  field(config.enable_detailed_results, "enable_detailed_results", "bool");
  field(
    config.enable_partial_relinearization_check, "enable_partial_relinearization_check", "bool");
  field(config.initial_position_sigma, "initial_position_sigma", "m");
  field(config.initial_rotation_yaw_sigma_deg, "initial_rotation_yaw_sigma_deg", "deg");
  field(
    config.initial_rotation_pitch_roll_sigma_deg, "initial_rotation_pitch_roll_sigma_deg", "deg");
  field(config.initial_velocity_sigma, "initial_velocity_sigma", "m/s");
  field(config.initial_bias_acc_sigma, "initial_bias_acc_sigma", "m/s^2");
  field(config.initial_bias_gyro_sigma, "initial_bias_gyro_sigma", "rad/s");
  field(config.initial_gravity_sigma, "initial_gravity_sigma", "m/s^2");
  field(config.additional_update_iterations, "additional_update_iterations", "num");

  check(config.lag, GT, 0.0, "lag");
  check(config.wildfire_threshold, GT, 0.0, "wildfire_threshold");
  check(config.relinearize_threshold_translation, GE, 0.0, "relinearize_threshold_translation");
  check(config.relinearize_threshold_rotation, GE, 0.0, "relinearize_threshold_rotation");
  check(config.relinearize_threshold_velocity, GE, 0.0, "relinearize_threshold_velocity");
  check(config.relinearize_threshold_bias_acc, GE, 0.0, "relinearize_threshold_bias_acc");
  check(config.relinearize_threshold_bias_gyro, GE, 0.0, "relinearize_threshold_bias_gyro");
  check(config.relinearize_skip, GE, 1, "relinearize_skip");
  check(config.initial_position_sigma, GE, 0.0, "initial_position_sigma");
  check(config.initial_rotation_yaw_sigma_deg, GE, 0.0, "initial_rotation_yaw_sigma_deg");
  check(
    config.initial_rotation_pitch_roll_sigma_deg, GE, 0.0, "initial_rotation_pitch_roll_sigma_deg");
  check(config.initial_velocity_sigma, GE, 0.0, "initial_velocity_sigma");
  check(config.initial_bias_acc_sigma, GE, 0.0, "initial_bias_acc_sigma");
  check(config.initial_bias_gyro_sigma, GE, 0.0, "initial_bias_gyro_sigma");
  checkCondition(
    config.factorization == "QR" || config.factorization == "CHOLESKY",
    "factorization must be QR or CHOLESKY");
}

void declare_config(ManagerConfig & config)
{
  using namespace config;
  name("Graph Manager Config");

  field(config.logs_directory, "logs_directory", "directory_path");
  field(config.world_frame, "world_frame", "str");
  field(config.navigation_frame, "navigation_frame", "str");
  field(config.body_frame, "body_frame", "str");

  {
    NameSpace ns("imu");
    field(config.T_B_I, "T_B_S", "gtsam::Pose3");
  }

  {
    NameSpace ns("graph/manager");
    field(config.log_level, "log_level", "trace|debug|info|warn|error|critical");
    field(config.max_ts_diff_for_imu_breaking, "max_ts_diff_for_imu_breaking", "s");
    field(config.max_measurement_latency, "max_measurement_latency", "s");
    field(config.smoother, "smoother");
  }

  check(config.body_frame, NE, config.world_frame, "body_frame");
}

}  // namespace graph
}  // namespace mimosa
