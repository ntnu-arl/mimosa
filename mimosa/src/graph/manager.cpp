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

  resetKey();

  // Setup trajectory logger
  trajectory_logger_ = createLogger(
    config_.logs_directory + "graph_manager_odometry.tum", "graph::Manager::odometry", "trace", false);
  trajectory_logger_->set_pattern("%v");

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

std::vector<double> Manager::getSmootherTimestampsNoLock()
{
  // Get all unique timestamps from the smoother in ascending order
  std::vector<double> timestamps;
#if SMOOTHER_IFL
  auto kt_map = smoother_->timestamps();
  timestamps.reserve(kt_map.size());
  for (const auto & kt_pair : kt_map) {
    timestamps.push_back(kt_pair.second);
  }
  std::sort(timestamps.begin(), timestamps.end());
  timestamps.erase(std::unique(timestamps.begin(), timestamps.end()), timestamps.end());
#else
  logger_->warn(
    "Getting smoother timestamps is only supported with IncrementalFixedLagSmoother. If using with "
    "ISAM2, this function will return an empty vector.");
#endif
  return timestamps;
}

void Manager::rekeyOneStepFactors(
  const gtsam::NonlinearFactorGraph & one_step_factors, const gtsam::Key new_key,
  gtsam::NonlinearFactorGraph & rekeyed_one_step_factors)
{
  if (one_step_factors.size()) {
    // Construct the mapping of the one step factor to the new key
    // This will always be:
    // X(0) -> X(key)
    // V(0) -> V(key)
    // B(0) -> B(key)
    std::map<gtsam::Key, gtsam::Key> key_mapping;
    key_mapping[X(0)] = X(new_key);
    key_mapping[V(0)] = V(new_key);
    key_mapping[B(0)] = B(new_key);

    // Rekey the one step factors to the new key
    for (const auto & f : one_step_factors) {
      auto rf = f->rekey(key_mapping);
      rekeyed_one_step_factors.push_back(rf);
    }
  }
}

Manager::DeclarationResult Manager::declare(
  const double ts, gtsam::Key & key, const bool use_to_init,
  const gtsam::NonlinearFactorGraph & one_step_factors)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);

  key = getNextKey();

  logger_->info("Declaring for ts: {} key: {}", ts, gdkf(key));

  if (!initialized_) {
    if (!use_to_init) {
      resetKey();
      return DeclarationResult::FAILURE_CANNOT_INIT_ON_MODALITY;
    }
    // Try to initialize
    gtsam::Rot3 R_W_B;
    V3D estimated_acc_bias, estimated_gyro_bias;
    if (!imu_manager_->estimateAttitude(R_W_B, estimated_acc_bias, estimated_gyro_bias)) {
      logger_->debug("Failed to estimate attitude");
      resetKey();
      return DeclarationResult::FAILURE_ATTITUDE_ESTIMATION;
    }

    // TODO: Handle initialization for radar.

    logger_->info("Estimated R_W_B:\n{}", R_W_B.matrix());
    logger_->info("Estimated acc bias: {}", estimated_acc_bias.transpose());
    logger_->info("Estimated gyro bias: {}", estimated_gyro_bias.transpose());

    const auto imu_bias = gtsam::imuBias::ConstantBias(estimated_acc_bias, estimated_gyro_bias);
    const gtsam::Pose3 T_W_Bk = gtsam::Pose3(R_W_B, V3D::Zero());

    // * Right now, the intialization is always without the rekeyed factors. Since the only other modality is the radar,
    // * which would only help in the case of non stationary initialization. Non-stationary initialization is not supported yet.
    initializeGraph(ts, key, T_W_Bk, V3D::Zero(), imu_bias);
    initialized_ = true;
    logger_->info("Graph initialized at ts: {} key: {}", ts, gdkf(key));
    return DeclarationResult::SUCCESS_INITIALIZED;
  }

  // The measurement can be added if:
  // - There exist IMU measurements between the new timestamp the timestamps of the states in the graph
  // - The timestamp of the new measurement is so close to a state in the graph that it can be considered to be at the same time
  auto smoother_timestamps = getSmootherTimestampsNoLock();
  logger_->trace("Lag contains: [{}, {}]", smoother_timestamps.front(), smoother_timestamps.back());

  // Print out the last 10 timestamps along with keys in the lag for debugging
  if (logger_->should_log(spdlog::level::trace)) {
    for (size_t i = (smoother_timestamps.size() > 10 ? smoother_timestamps.size() - 10 : 0);
         i < smoother_timestamps.size(); ++i) {
      const auto t = smoother_timestamps[i];
      logger_->trace("Lag ts: {}, key: {}", t, ts_key_map_[t]);
    }
  }

  const auto latency = ts - smoother_timestamps.back(); // Hence negative means out of order
  logger_->trace("Measurement latency wrt latest state: {}", latency);
  
  if (latency < -config_.max_measurement_latency) {
    logger_->warn(
      "Timestamp {} is older than max measurement latency ({} s). Ignoring measurement", 
      ts, config_.max_measurement_latency);
    discardCurrentKey();
    return DeclarationResult::FAILURE_OLDER_THAN_MAX_LATENCY;
  }

  // Lower bound returns the first element that is not less than ts
  auto it = std::lower_bound(smoother_timestamps.begin(), smoother_timestamps.end(), ts);

  if (it == smoother_timestamps.begin()) {
    if (ts_key_map_.size() == 1)
    {
      logger_->debug("Measurement is before the initialization state. Ignoring measurement");
      discardCurrentKey();
      return DeclarationResult::FAILURE_OLDER_THAN_INITIALIZATION;
    }
    // New measurement is before the earliest state - no way to handle this
    logger_->warn("Timestamp is older than entire lag window. Ignoring measurement");
    discardCurrentKey();
    return DeclarationResult::FAILURE_OLDER_THAN_LAG;
  }

  if (it != smoother_timestamps.end()) {
    // smoother_timestamps.front() < ts <= smoother_timestamps.back()

    // Now we get the previous and next states of the timestamp ts such that
    // prev_ts < ts <= next_ts
    const auto next_ts = *it;
    auto prev_it = it;
    prev_it--;
    const auto prev_ts = *prev_it;

    logger_->trace("prev_ts: {}, prev_key: {}, next_ts: {}, next_key: {}", prev_ts, ts_key_map_[prev_ts], next_ts, ts_key_map_[next_ts]);

    // If there are enough IMU measurements on both sides, then it is feasible to break the current IMU factor
    bool use_same_key = false;

    auto n_imu_before = imu_manager_->getNumMeasurementsBetween(prev_ts, ts);
    if (n_imu_before < 2) {
      logger_->debug("Using same key [prev] due to no imu in between. ts_diff: {}", ts - prev_ts);
      key = ts_key_map_[prev_ts];
      use_same_key = true;
    }

    auto n_imu_after = imu_manager_->getNumMeasurementsBetween(ts, next_ts);
    if (n_imu_after < 2) {
      logger_->debug("Using same key [next] due to no imu in between. ts_diff: {}", next_ts - ts);
      key = ts_key_map_[next_ts];
      use_same_key = true;
    }

    // At this point we know what key is to be used for sure.
    gtsam::NonlinearFactorGraph rekeyed_one_step_factors;
    rekeyOneStepFactors(one_step_factors, key, rekeyed_one_step_factors);

    if (use_same_key) {
      discardCurrentKey();

      // If there are one step factors then this is their define
      if (rekeyed_one_step_factors.size()) {
        logger_->debug("Defining with same key and one step factors");
        auto unused_values = gtsam::Values();
        defineNoLock(rekeyed_one_step_factors, unused_values, DeclarationResult::SUCCESS_SAME_KEY);
      }
      return DeclarationResult::SUCCESS_SAME_KEY;
    }

    // Now we actually do need to break the IMU factor
    Stopwatch sw;
    logger_->debug("Breaking IMU with n_imu_before: {} and n_imu_after: {}", n_imu_before, n_imu_after);

    State prev_state;
    auto prev_key = ts_key_map_[prev_ts];
    prev_state.update(
      prev_key, prev_ts,
      gtsam::NavState(
        smoother_->calculateEstimate<gtsam::Pose3>(X(prev_key)),
        smoother_->calculateEstimate<V3D>(V(prev_key))),
      smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(prev_key)),
      smoother_->calculateEstimate<gtsam::Unit3>(G(0)));

    State next_state;
    auto next_key = ts_key_map_[next_ts];
    next_state.update(
      next_key, next_ts,
      gtsam::NavState(
        smoother_->calculateEstimate<gtsam::Pose3>(X(next_key)),
        smoother_->calculateEstimate<V3D>(V(next_key))),
      smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(next_key)),
      smoother_->calculateEstimate<gtsam::Unit3>(G(0)));

    logger_->trace("States estimated");

    gtsam::NonlinearFactorGraph new_factors;

    // Get the propagated state and IMU factor
    gtsam::NavState propagated_state;
    imu_manager_->addImuFactorAndGetNavState(prev_state, ts, key, new_factors, propagated_state);
    auto & imu_factor_1 = *new_factors.end()[-2];
    auto & bias_between_factor_1 = *new_factors.end()[-1];

    new_factors.add(rekeyed_one_step_factors);

    // Add the new state to the smoother
    gtsam::Values values;
    values.insert(X(key), propagated_state.pose());
    values.insert(V(key), propagated_state.velocity());
    values.insert(B(key), prev_state.imuBias());

    gtsam::IncrementalFixedLagSmoother::KeyTimestampMap key_ts_map;
    for (const auto & [k, value] : values) {
      key_ts_map[k] = ts;
    }

    // Add an imu factor between the current and the next state
    imu_manager_->addImuFactor(
      ts, next_state.ts(), prev_state.imuBias(), key, next_state.key(), new_factors);
    auto & imu_factor_2 = *new_factors.end()[-2];
    auto & bias_between_factor_2 = *new_factors.end()[-1];

    // Find the imu factor that connects the previous state to the next state

#if SMOOTHER_IFL
    const auto factors_in_graph = smoother_->getFactors();
#else
    const auto factors_in_graph = smoother_->getFactorsUnsafe();
#endif
    // Iterate backwards in the factors to find the imu factor that connects the previous state to the current state
    gtsam::FactorIndices factors_to_remove;
    bool imu_factor_found = false;
    bool bias_between_factor_found = false;
    for (int i = factors_in_graph.size() - 1; i >= 0; --i) {
      try {
        if (!factors_in_graph[i]) continue;

        const auto f = factors_in_graph[i];
        if (!imu_factor_found) {
          // Look for the imu factor
          if (
            f->dim() == imu_factor_1.dim() && f->keys().size() == imu_factor_1.keys().size() &&
            f->keys()[0] == imu_factor_1.keys()[0] && f->keys()[2] == imu_factor_2.keys()[2]) {
            if (logger_->should_log(spdlog::level::trace)) {
              logger_->trace(
                "Found imu factor to remove connecting keys {} and {}",
                gdkf(imu_factor_1.keys()[0]),
                gdkf(imu_factor_2.keys()[2]));
              f->print();
            }
            factors_to_remove.push_back(i);
            imu_factor_found = true;
          }
        }
        if (!bias_between_factor_found) {
          // Look for the between factor
          if (
            f->dim() == bias_between_factor_1.dim() &&
            f->keys().size() == bias_between_factor_1.keys().size() &&
            f->keys()[0] == bias_between_factor_1.keys()[0] &&
            f->keys()[1] == bias_between_factor_2.keys()[1]) {
            if (logger_->should_log(spdlog::level::trace)) {
              logger_->trace(
                "Found bias between factor to remove connecting keys {} and {}",
                gdkf(bias_between_factor_1.keys()[0]),
                gdkf(bias_between_factor_2.keys()[1]));
              f->print();
            }
            factors_to_remove.push_back(i);
            bias_between_factor_found = true;
          }
        }
      } catch (const std::exception & e) {
        std::cerr << e.what() << '\n';
        throw;
      }
    }

    if (!imu_factor_found) {
      logger_->error("Could not find the imu factor that connects the previous state to the current state");
    }
    if (!bias_between_factor_found) {
      logger_->error("Could not find the bias between factor that connects the previous state to the current state");
    }
    if (!imu_factor_found || !bias_between_factor_found) {
      logger_->error("Cannot handle out of order measurement as the required factors to break cannot be found. Discarding this measurement");
      discardCurrentKey();
      return DeclarationResult::FAILURE_CANNOT_HANDLE_OUT_OF_ORDER;
    }

    logger_->trace("New factors being added: (total {})", new_factors.size());
    if (logger_->should_log(spdlog::level::trace)) {
      for (const auto & f : new_factors) {
        f->print();
      }
    }

#if SMOOTHER_IFL
    smoother_->update(new_factors, values, key_ts_map, factors_to_remove);
#else
    smoother_->update(new_factors, values, factors_to_remove);
#endif

    ts_key_map_[ts] = key;  // This is the key without the character
    optimized_values_ = smoother_->calculateEstimate();

    // Update the current state
    updateStateToKeyTs(state_.key(), state_.ts());

    if (rekeyed_one_step_factors.size()) {
      imu_manager_->setPropagationBaseState(state_);
      // Not publishing results here as it would be for the same key and ts
    }

    logger_->debug("Handled out of order measurement in {} ms", sw.elapsedMs());
    return DeclarationResult::SUCCESS_OUT_OF_ORDER;
  }

  // ts > smoother_timestamps.back()
  logger_->trace("Normal condition: ts is after the latest state in the lag");
  // If IMU exists, use it to add a state
  // If IMU does not exist, collapse into current state
  size_t n_imu_bw = imu_manager_->getNumMeasurementsBetween(state_.ts(), ts);
  if (n_imu_bw < 2) {
    // Collapse into same key
    key = state_.key();
    logger_->info("Using same key: {} with the ts_diff: {}", gdkf(key), ts - state_.ts());
    discardCurrentKey();
    gtsam::NonlinearFactorGraph rekeyed_one_step_factors;
    rekeyOneStepFactors(one_step_factors, key, rekeyed_one_step_factors);
    // If there are one step factors then this is their define
    if (rekeyed_one_step_factors.size()) {
      logger_->debug("Defining with same key and one step factors");
      auto unused_values = gtsam::Values();
      defineNoLock(rekeyed_one_step_factors, unused_values, DeclarationResult::SUCCESS_SAME_KEY);
    }
    return DeclarationResult::SUCCESS_SAME_KEY;
  }

  gtsam::NonlinearFactorGraph rekeyed_one_step_factors;
  rekeyOneStepFactors(one_step_factors, key, rekeyed_one_step_factors);

  // Create a new state at ts
  // Add a preintegrated imu factor between the current state and this new state
  // Add the new state to the smoother
  // Optimize

  logger_->trace("Adding imu factor using {} measurements", n_imu_bw);

  // Get the propagated state and imu factor
  gtsam::NavState propagated_state;
  gtsam::NonlinearFactorGraph new_factors;
  imu_manager_->addImuFactorAndGetNavState(state_, ts, key, new_factors, propagated_state);
  // Add the rekeyed one step factors to the new factors
  new_factors.add(rekeyed_one_step_factors);

  logger_->trace("Added imu factor");
  // Add the new state to the smoother
  gtsam::Values values;
  values.insert(X(key), propagated_state.pose());
  values.insert(V(key), propagated_state.velocity());
  values.insert(B(key), state_.imuBias());

  logger_->trace("Before smoother update");
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
  logger_->trace("After smoother update");

  ts_key_map_[ts] = key;  // This is the key without the character
  optimized_values_ = smoother_->calculateEstimate();
  logger_->trace("Calculated optimized values with {} values", optimized_values_.size());

  // Update the current state
  updateStateToKeyTs(key, ts);

  if (rekeyed_one_step_factors.size()) {
    // These difference values are only valid when there is a propagation and correction step
    const auto p = smoother_->calculateEstimate<gtsam::Pose3>(X(key));
    const auto pose_diff = p.between(propagated_state.pose());
    debug_msg_.diff_against_imu_prior_trans_cm = pose_diff.translation().norm() * 100;
    debug_msg_.diff_against_imu_prior_rot_deg = rad2deg(pose_diff.rotation().axisAngle().second);
    const auto v = smoother_->calculateEstimate<V3D>(V(key));
    debug_msg_.diff_against_imu_prior_vel_cm_p_s = (v - propagated_state.velocity()).norm() * 100; // Convert to cm/s

    imu_manager_->setPropagationBaseState(state_);
    publishResults();
  }
  return Manager::DeclarationResult::SUCCESS_NORMAL;
}

void Manager::getCurrentState(State & state)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  state = state_;
}

void Manager::getStateUpto(const double ts, State & state)
{
  logger_->trace("Waiting to lock in getStateUpto for ts: {}", ts);
  std::lock_guard<std::mutex> lock(graph_mutex_);
  logger_->trace("Locked in getStateUpto");
  getStateUptoNoLock(ts, state);
  logger_->trace("unlocked in getStateUpto");
}

void Manager::getStateUptoNoLock(const double ts, State & state)
{
  logger_->trace("Getting state upto ts: {}", ts);
  if (ts_key_map_.empty()) {
    // This means that there are no states added yet. This only happens before initialization
    if (initialized_)
    {
      logger_->error("ts_key_map_ is empty and initialized_ is true. This should not never happen");
    }
    return;
  }

  auto it = ts_key_map_.upper_bound(ts);
  if (it == ts_key_map_.begin()) {
    // This means that the timestamp is older than all the states that have ever been added

    // If the difference between the timestamps is small, we can just warn and use the new state
    if (std::abs(ts - ts_key_map_.begin()->first) > 4e-3) {  // 4ms
      logCriticalException<std::runtime_error>(
        logger_, fmt::format(
                   "No state available upto ts: {} the only ts in there is {} . Size of map: {}",
                   ts, ts_key_map_.begin()->first, ts_key_map_.size()));
    } else {
      logger_->warn(
        "Timestamp is older than all states ever, but within 4ms of the first state. Using the "
        "first state.");
    }
  } else {
    --it;
  }

#if SMOOTHER_IFL
  // If we are using the fixed lag smoother, then it is possible that the state has been removed from the
  // smoother. Hence, we need to check if the key exists in the smoother
  const auto & smoother_timestamps = smoother_->timestamps();
  if (smoother_timestamps.find(X(it->second)) == smoother_timestamps.end()) {
    logger_->error(
      "The requested state at ts: {} with key: {} has been removed from the smoother. Failing "
      "silently assuming that the caller can handle this.",
      it->first, gdkf(it->second));
    return;
  }
#endif

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
  std::lock_guard<std::mutex> lock(graph_mutex_);
  defineNoLock(graph, optimized_values, result);
}

void Manager::defineNoLock(
  const gtsam::NonlinearFactorGraph & graph, gtsam::Values & optimized_values,
  const DeclarationResult result)
{
  // All the factors in the graph are attached to variables that already exist in the smoother
  // Hence, we can just add the graph to the smoother
  logger_->trace("Before smoother update with {} factors", graph.size());
  smoother_->update(graph);
  for (int i = 0; i < config_.smoother.additional_update_iterations; ++i) {
    smoother_->update();
  }
  logger_->trace("After smoother update");

  if (result != DeclarationResult::SUCCESS_SAME_KEY) {
    // ? In the case of same key, the optimized values returned are empty for some reason
    optimized_values_ = smoother_->calculateEstimate();

    logger_->trace("Calculated optimized values with {} values", optimized_values_.size());

    // These difference values are only valid when there is a propagation and correction step
    const auto p = optimized_values_.at<gtsam::Pose3>(X(state_.key()));
    const auto pose_diff = p.between(state_.navState().pose());
    debug_msg_.diff_against_imu_prior_trans_cm = pose_diff.translation().norm() * 100;
    debug_msg_.diff_against_imu_prior_rot_deg = rad2deg(pose_diff.rotation().axisAngle().second);
    const auto v = optimized_values_.at<V3D>(V(state_.key()));
    debug_msg_.diff_against_imu_prior_vel_cm_p_s = (v - state_.navState().velocity()).norm() * 100; // Convert to cm/s
  }
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
  logger_->trace(
    "Updating state to key: {} at ts: {} previous key: {} previous ts: {}",
    gdkf(key), ts, gdkf(state_.key()), state_.ts());

  // Update the state to be at key
  const auto p = smoother_->calculateEstimate<gtsam::Pose3>(X(key));
  const auto v = smoother_->calculateEstimate<V3D>(V(key));
  const auto b = smoother_->calculateEstimate<gtsam::imuBias::ConstantBias>(B(key));
  const auto g = smoother_->calculateEstimate<gtsam::Unit3>(G(0));

  state_.update(key, ts, gtsam::NavState(p, v), b, g);
}

void Manager::initializeGraph(
  const double ts, const gtsam::Key key, const gtsam::Pose3 & T_W_B, const V3D & vel,
  const gtsam::imuBias::ConstantBias & bias, const gtsam::NonlinearFactorGraph & additional_factors)
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

  graph.add(additional_factors);

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
    tf2_broadcaster_, state_.navState().pose(), config_.map_frame, config_.body_frame,
    state_.ts());

  // Broadcast the nav to map transform
  const V3D n_g_direction = -V3D::UnitZ();
  V3D w_g_direction = state_.gravity().unitVector();
  // n_g = T_N_W * w_g
  gtsam::Pose3 T_N_W = gtsam::Pose3(
    gtsam::Rot3(Eigen::Quaterniond().setFromTwoVectors(w_g_direction, n_g_direction)), V3D::Zero());
  broadcastTransform(
    tf2_static_broadcaster_, T_N_W, config_.navigation_frame, config_.map_frame, 0.0);

  static nav_msgs::Path path;
  path.header.frame_id = config_.map_frame;
  publishPath(pub_path_, path, state_.navState().pose(), state_.ts());

  // Write the trajectory to the trajectory logger
  // timestamp tx ty tz qx qy qz qw
  const gtsam::Quaternion q = state_.navState().pose().rotation().toQuaternion();
  trajectory_logger_->info(
    "{} {} {} {} {} {} {} {}", state_.ts(), state_.navState().pose().translation().x(),
    state_.navState().pose().translation().y(), state_.navState().pose().translation().z(), q.x(),
    q.y(), q.z(), q.w());

  // Publish odometry
  static nav_msgs::Odometry odometry;
  if (pub_odometry_.getNumSubscribers()) {
    odometry.header.frame_id = config_.map_frame;
    odometry.child_frame_id = config_.body_frame;
    odometry.header.stamp.fromSec(state_.ts());
    convert(state_.navState().pose(), odometry.pose.pose);
    convert(state_.navState().velocity(), odometry.twist.twist.linear);
    pub_odometry_.publish(odometry);
  }

  // Publish transform stamped
  static geometry_msgs::TransformStamped ts_frame_child;
  if (pub_transform_stamped_.getNumSubscribers()) {
    ts_frame_child.header.frame_id = config_.map_frame;
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
  opt_path.header.frame_id = config_.map_frame;
  for (const auto & [key, value] : optimized_values_) {
    // Check if key is of pose type
    if (gtsam::Symbol(key).chr() != 'x') {
      continue;
    }

    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = config_.map_frame;
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
  field(config.map_frame, "map_frame", "str");
  field(config.navigation_frame, "navigation_frame", "str");
  field(config.body_frame, "body_frame", "str");

  {
    NameSpace ns("graph/manager");
    field(config.log_level, "log_level", "trace|debug|info|warn|error|critical");
    field(config.max_measurement_latency, "max_measurement_latency", "s");
    field(config.smoother, "smoother");
  }

  check(config.body_frame, NE, config.map_frame, "body_frame");
}

}  // namespace graph
}  // namespace mimosa
