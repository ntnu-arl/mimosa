// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/lidar/manager.hpp"

namespace mimosa
{
namespace lidar
{
Manager::Manager(
  ros::NodeHandle & pnh, mimosa::imu::Manager::Ptr imu_manager,
  mimosa::graph::Manager::Ptr graph_manager)
: imu_manager_(imu_manager),
  graph_manager_(graph_manager),
  config_(config::checkValid(config::fromRos<ManagerConfig>(pnh))),
  z_offset_(-config_.lidar_to_sensor_transform[11] / 1000.0),
  range_min_sq_(config_.range_min * config_.range_min),
  range_max_sq_(config_.range_max * config_.range_max)
{
  logger_ =
    createLogger(config_.logs_directory + "lidar_manager.log", "lidar::Manager", config_.log_level);
  logger_->info("lidar::Manager initialized with params:\n {}", config::toString(config_));

  geometric_ = std::make_unique<Geometric>(pnh);
  photometric_ = std::make_unique<Photometric>(pnh);

  // Checks that span across configs
  if (!config_.create_full_res_pointcloud && photometric_->config.enabled) {
    logCriticalException<std::runtime_error>(
      logger_, "Photometric being enabled requires create_full_res_pointcloud to be true.");
  }

  initialized_ = false;

  pub_points_ = pnh.advertise<sensor_msgs::PointCloud2>("lidar/manager/points_full_res", 1);
  pub_debug_ = pnh.advertise<mimosa_msgs::LidarManagerDebug>("lidar/manager/debug", 1);

  // Setup trajectory logger
  trajectory_logger_ = createLogger(
    config_.logs_directory + "lidar_manager_odometry.log", "lidar::Manager", "trace", false);
  trajectory_logger_->set_pattern("%v");

  if (config_.enabled) {
    sub_points_ = pnh.subscribe("lidar/manager/points_in", 5, &Manager::callback, this);
  }
}

void Manager::callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  if (!config_.enabled) {
    return;
  }
  Stopwatch sw;
  debug_msg_.initialized = initialized_;
  // Drop the initial few pointclouds
  static int initial_skip = config_.initial_skip;
  if (initial_skip > 0) {
    --initial_skip;
    logger_->info("Dropping initial pointcloud. {} more will be dropped", initial_skip);
    return;
  }

  // Current pointcloud must be newer than the previous one
  static double prev_header_ts = 0.0;
  header_ts_ = msg->header.stamp.toSec() + config_.ts_offset;
  if (header_ts_ <= prev_header_ts) {
    logger_->error(
      "Skipping pointcloud with timestamp {} because it is not newer than previous timestamp {}. "
      "This should never happen. Check the header timestamps of the lidar pointclouds",
      header_ts_, prev_header_ts);
    return;
  }
  prev_header_ts = header_ts_;

  // Preprocess the pointcloud
  static const PType point_type = decodePointType(msg->fields);
  switch (point_type) {
    case PType::Ouster:
      prepareInput<PointOuster>(msg);
      break;
    case PType::OusterR8:
      prepareInput<PointOusterR8>(msg);
      break;
    case PType::Hesai:
      prepareInput<PointHesai>(msg);
      break;
    case PType::Livox:
      prepareInput<PointLivox>(msg);
      break;
    case PType::LivoxFromCustom2:
      prepareInput<PointLivoxFromCustom2>(msg);
      break;
    case PType::Velodyne:
      prepareInput<PointVelodyne>(msg);
      break;
    case PType::Rslidar:
      prepareInput<PointRslidar>(msg);
      break;
    default:
      throw std::runtime_error("Unsupported point type");
  }

  graph_manager_->getStateUpto(header_ts_, prev_state_);

  logger_->debug("Declaring (ts: {})", corrected_ts_);
  Stopwatch sw_declare;
  graph::Manager::DeclarationResult result =
    graph_manager_->declare(corrected_ts_, new_key_, config_.use_to_init);
  // Handle possible failures
  switch (result) {
    case graph::Manager::DeclarationResult::FAILURE_CANNOT_INIT_ON_MODALITY:
      logger_->info("Cannot initialize on this modality. Skipping this pointcloud");
      return;

    case graph::Manager::DeclarationResult::FAILURE_ATTITUDE_ESTIMATION:
      logger_->debug(
        "Graph manager could not initialize due to attitude estimation failure. Skipping this "
        "pointcloud");
      return;

    case graph::Manager::DeclarationResult::FAILURE_OLDER_THAN_INITIALIZATION:
      logger_->warn(
        "timestamp is older than the graph initialization timestamp. If this "
        "happens continously, check the time sychronization between the sensors");
      return;

    case graph::Manager::DeclarationResult::FAILURE_OLDER_THAN_LAG:
      logger_->error("Timestamp is older than entire lag window. Ignoring measurement");
      return;

    case graph::Manager::DeclarationResult::FAILURE_OLDER_THAN_MAX_LATENCY:
      logger_->warn("Timestamp is too old to be considered. Ignoring measurement");
      return;

    case graph::Manager::DeclarationResult::SUCCESS_INITIALIZED:
    case graph::Manager::DeclarationResult::SUCCESS_SAME_KEY:
    case graph::Manager::DeclarationResult::SUCCESS_OUT_OF_ORDER:
    case graph::Manager::DeclarationResult::SUCCESS_NORMAL:
      break;

    default:
      logCriticalException<std::logic_error>(
        logger_, fmt::format(
                   "Unknown declaration result. Maybe the graph::manager::DeclarationResult API "
                   "has been updated. Received: {}",
                   result));
  }
  debug_msg_.t_declare = sw_declare.elapsedMs();

  if (imu_preintegrator_ == nullptr) {
    imu_preintegrator_ =
      std::make_unique<gtsam::PreintegratedImuMeasurements>(imu_manager_->getPreintegratorParams());
  }

  deskewPoints();

  preprocess(X(new_key_));

  static bool first = true;
  gtsam::Pose3 T_W_Bk_opt;
  static gtsam::Values opt_values;
  if (first) {
    first = false;
    broadcastTransform(
      tf2_static_broadcaster_, config_.T_B_S, config_.body_frame, config_.sensor_frame, header_ts_);

    initialized_ = true;

    logger_->info("Initialized");

    T_W_Bk_opt = graph_manager_->getPoseAt(new_key_);
    opt_values.insert(X(new_key_), T_W_Bk_opt);
  } else {
    // Get the new factors
    gtsam::Values initial_values = opt_values;
    initial_values.insert_or_assign(X(new_key_), propagated_state_.pose());
    gtsam::NonlinearFactorGraph new_factors;
    getFactors(initial_values, new_factors);

    define(new_factors, opt_values, result);

    // const gtsam::NonlinearFactorGraph factors = graph_manager_->getFactors();
    // photometric_->visualizeTracks(
    //   factors, opt_values, config_.world_frame, corrected_ts_, X(new_key_));

    T_W_Bk_opt = opt_values.at<gtsam::Pose3>(X(new_key_));
  }

  postDefineUpdate(X(new_key_), opt_values);

  publishResults(T_W_Bk_opt);

  debug_msg_.header.stamp.fromSec(corrected_ts_);
  debug_msg_.t_full = sw.elapsedMs();
  pub_debug_.publish(debug_msg_);
}

template <typename PointT>
void Manager::prepareInput(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  Stopwatch sw;

  // Convert the pointcloud to pcl
  pcl::PointCloud<PointT> msg_cloud;
  toPcl(*msg, msg_cloud);
  debug_msg_.n_points_in = msg_cloud.size();
  debug_msg_.t_pcl_conversion = sw.tickMs();

  // Iterate over the points to select the final points to be used
  // Clear previous points
  points_full_.clear();
  geometric_point_idxs_.clear();
  ns_idx_pairs_.clear();
  unique_ns_.clear();
  idxs_at_unique_ns_.clear();

  // Reserve space for the points
  points_full_.reserve(msg_cloud.size());
  geometric_point_idxs_.reserve(msg_cloud.size());
  ns_idx_pairs_.reserve(msg_cloud.size());
  unique_ns_.reserve(msg_cloud.size());
  idxs_at_unique_ns_.reserve(msg_cloud.size());
  debug_msg_.t_clear_reserve = sw.tickMs();

  uint32_t last_point_ns = std::numeric_limits<uint32_t>::min();
  const size_t skip_divisor =
    config_.create_full_res_pointcloud ? 1 : geometric_->config.point_skip_divisor;
  for (size_t i = 0; i < msg_cloud.size(); i = i + skip_divisor) {
    // This loop will always be sequential
    const PointT & pin = msg_cloud[i];

    // Reject unreliable points
    // NaN filter
    if (std::isnan(pin.x) || std::isnan(pin.y) || std::isnan(pin.z)) continue;

    // Livox tag filter
    if constexpr (
      std::is_same<PointT, PointLivox>::value ||
      std::is_same<PointT, PointLivoxFromCustom2>::value) {
      if (!((pin.tag & 0x30) == 0x10 || (pin.tag & 0x30) == 0x00)) {
        continue;
      }
    }

    // Intensity filter
    if (pin.intensity < config_.intensity_min || pin.intensity > config_.intensity_max) continue;

    // Range filter
    float range_sq = pin.x * pin.x + pin.y * pin.y + pin.z * pin.z;
    if (range_sq < range_min_sq_ || range_sq > range_max_sq_) continue;

    // Decode time to be nanoseconds since the header timestamp
    uint32_t t_ns;
    if constexpr (
      std::is_same<PointT, PointOuster>::value || std::is_same<PointT, PointOusterR8>::value) {
      t_ns = pin.t;
    } else if constexpr (std::is_same<PointT, PointHesai>::value) {
      t_ns = (pin.timestamp - header_ts_) * 1e9;
    } else if constexpr (std::is_same<PointT, PointLivox>::value) {
      t_ns = pin.timestamp - header_ts_ * 1e9;

      // Additional sensor specific processing
      if (t_ns >= 1e8 - 2e6)  // 100ms - 2ms
      {
        // The mid360 starts the next pointcloud at a time less than 100ms from the start of the pointcloud
        // this causes issues in getting the state since we assume that the timestamp of the first point in
        // pointcloud k is greater than the timestamp of the last point in pointcloud k-1. The issue manifests in
        // the getStateUpto function called in the deskeewPoints function
        continue;
      }
    } else if constexpr (std::is_same<PointT, PointLivoxFromCustom2>::value) {
      t_ns = pin.t;
    } else if constexpr (std::is_same<PointT, PointVelodyne>::value) {
      t_ns = pin.time * 1e9;

      // Additional sensor specific processing
      if (t_ns >= 1e8 - 2e6)  // 100ms - 2ms
      {
        // The velodyne starts the next pointcloud at a time less than 100ms from the start of the pointcloud
        // this causes issues in getting the state since we assume that the timestamp of the first point in
        // pointcloud k is greater than the timestamp of the last point in pointcloud k-1. The issue manifests in
        // the getStateUpto function called in the deskeewPoints function
        continue;
      }
    } else if constexpr (std::is_same<PointT, PointRslidar>::value) {
      t_ns = (pin.timestamp - header_ts_) * 1e9;
    } else {
      throw std::runtime_error("Unsupported point type");
    }
    last_point_ns = std::max(last_point_ns, t_ns);

    points_full_.points.emplace_back(
      pin.x, pin.y, pin.z + z_offset_, pin.intensity, t_ns, i, std::sqrt(range_sq));
    const size_t new_idx = points_full_.size() - 1;
    ns_idx_pairs_.emplace_back(t_ns, new_idx);

    // Point Skip filter
    if (i % geometric_->config.point_skip_divisor != 0) continue;

    // Ring filter - does not make sense in a Livox pointcloud
    if constexpr (
      !std::is_same<PointT, PointLivox>::value &&
      !std::is_same<PointT, PointLivoxFromCustom2>::value) {
      if (pin.ring % geometric_->config.ring_skip_divisor != 0) continue;
    }

    geometric_point_idxs_.push_back(new_idx);
  }
  corrected_ts_ = globalTs(last_point_ns);
  debug_msg_.t_preprocess_filter = sw.tickMs();

  // Sort the ns_idx_pairs_ vector by timestamp
  std::sort(ns_idx_pairs_.begin(), ns_idx_pairs_.end(), [](const auto & a, const auto & b) {
    return a.first < b.first;
  });

  if (!ns_idx_pairs_.empty()) {
    // Start a new group for the first timestamp
    unique_ns_.push_back(ns_idx_pairs_[0].first);
    idxs_at_unique_ns_.emplace_back();
    idxs_at_unique_ns_.back().push_back(ns_idx_pairs_[0].second);
  }

  // Iterate over the timestamps and group the points by timestamp
  static size_t max_idxs_per_ns = 0;
  for (size_t i = 1; i < ns_idx_pairs_.size(); ++i) {
    const auto & [ns, idx] = ns_idx_pairs_[i];
    // Compare with previous timestamp
    if (ns == ns_idx_pairs_[i - 1].first) {
      // Same timestamp, so push into the current group
      idxs_at_unique_ns_.back().push_back(idx);
    } else {
      // Different timestamp => start a new group
      max_idxs_per_ns = std::max(max_idxs_per_ns, idxs_at_unique_ns_.back().size());

      unique_ns_.push_back(ns);
      idxs_at_unique_ns_.emplace_back();
      idxs_at_unique_ns_.back().reserve(max_idxs_per_ns);
      idxs_at_unique_ns_.back().push_back(idx);
    }
  }

  debug_msg_.t_preprocess_sort = sw.tickMs();

  logger_->debug(
    "Filtering done, header ts: {}, corrected ts: {}, points_full_size: {}", header_ts_,
    corrected_ts_, points_full_.size());

  // Save a copy of the raw points. This is needed by photometric
  if (photometric_->config.enabled) {
    points_raw_.clear();
    points_raw_ = points_full_;
  }

  debug_msg_.t_preprocess = sw.elapsedMs();
}

void Manager::deskewPoints()
{
  logger_->debug("Deskewing points");
  Stopwatch sw;

  if (photometric_->config.enabled) {
    interpolated_map_T_Le_Lt_.clear();
    interpolated_map_T_Le_Lt_.reserve(unique_ns_.size());
  }

  std::vector<gtsam::Pose3> T_W_Bts_at_unique_ns;
  T_W_Bts_at_unique_ns.reserve(unique_ns_.size());

  if (!initialized_) {
    logger_->warn(
      "Not initialized, not deskewing pointcloud. This should only happen on the first pointcloud. "
      "If you are seeing this message later, then there is a bug");
    for (const auto ns : unique_ns_) {
      interpolated_map_T_Le_Lt_[ns] = gtsam::Pose3::Identity();
    }
    return;
  }

  State state = prev_state_;

  // if (abs(globalTs(unique_ns_.front()) - state.ts()) > 0.01) {
  //   logCriticalException<std::runtime_error>(
  //     logger_,
  //     fmt::format(
  //       "Issue with state timstamps. First timestamp: {} state_ts: {}, state_ts - first_ts: {}",
  //       globalTs(unique_ns_.front()), state.ts(), state.ts() - globalTs(unique_ns_.front())));
  // }
  // If the state timestamp is after the first point in the current pointcloud, then this needs to be corrected
  if (state.ts() > globalTs(unique_ns_.front())) {
    if ((state.ts() - globalTs(unique_ns_.front())) * 1000 > 1)  // 1ms
    {
      logger_->warn(
        "Moving state backwards in time making error of {}ms",
        (state.ts() - globalTs(unique_ns_.front())) * 1000);
    }

    state.update(
      state.key(), globalTs(unique_ns_.front()), state.navState(), state.imuBias(),
      state.gravity());
  }

  logger_->debug("Got state upto : {} with ts: {}", header_ts_, state.ts());
  // Preintegrate the imu measurements
  ImuBuffer imu_measurements;
  // Note that this should always be from the last state to the end of the current scan due to the preintegrator
  // Particularly, since the preintegrator always starts integrating from the second measurement, it assumes that the
  // first measurement is at the previous state. So the previous state might not be at the same timestamp as the header_ts_
  // of the current pointcloud
  imu_manager_->getInterpolatedMeasurements(state.ts(), corrected_ts_, imu_measurements);

  if (imu_measurements.size() < 2) {
    std::string msg = "Preintegration not possible as there are less than 2 measurements P1";
    logger_->error(msg);
    throw std::runtime_error(msg);
  }

  imu_preintegrator_->resetIntegrationAndSetBias(state.imuBias());

  // Integrate the measurements
  auto curr_itr = imu_measurements.begin();
  auto next_itr = curr_itr;
  ++next_itr;

  gtsam::NavState curr_state;
  propagated_state_ = state.navState();

  auto unique_ns_itr = unique_ns_.begin();
  for (; next_itr != imu_measurements.end(); ++curr_itr, ++next_itr) {
    double dt = next_itr->first - curr_itr->first;
    imu_preintegrator_->integrateMeasurement(
      curr_itr->second.head<3>(), curr_itr->second.tail<3>(), dt);

    curr_state = propagated_state_;
    propagated_state_ =
      imu_preintegrator_->predict(state.navState(), state.imuBias(), state.gravity());
    // Propagated state is at the time of the next_itr

    while (true) {
      if (unique_ns_itr == unique_ns_.end()) {
        break;
      }
      const auto ts = globalTs(*unique_ns_itr);
      if (ts > next_itr->first) {
        break;
      }

      const double delta_t = ts - curr_itr->first;
      V3D acc = state.imuBias().correctAccelerometer(curr_itr->second.head<3>());
      V3D omega = state.imuBias().correctGyroscope(curr_itr->second.tail<3>());

      if (imu_preintegrator_->params()->body_P_sensor) {
        std::tie(acc, omega) = imu_preintegrator_->correctMeasurementsBySensorPose(acc, omega);
      }

      // The curr_state should now be propagated with const a and omega for delta_t
      gtsam::Rot3 R_W_Bt = curr_state.pose().rotation() * gtsam::Rot3::Expmap(omega * delta_t);
      V3D p_W_Bt = curr_state.pose().translation() + curr_state.velocity() * delta_t +
                   0.5 * curr_state.pose().rotation().matrix() * acc * delta_t * delta_t +
                   0.5 * state.gravity() * imu_preintegrator_->params()->getGravity().norm() *
                     delta_t * delta_t;

      T_W_Bts_at_unique_ns.emplace_back(gtsam::Pose3(R_W_Bt, p_W_Bt));
      ++unique_ns_itr;
    }
  }

  // Deskew the points
  // Note the propagated state = T_W_Be
  const gtsam::Pose3 T_Le_W = config_.T_B_S.inverse() * propagated_state_.pose().inverse();

  for (size_t i = 0; i < unique_ns_.size(); ++i) {
    const gtsam::Pose3 T_Le_Lt = T_Le_W * T_W_Bts_at_unique_ns[i] * config_.T_B_S;

    if (photometric_->config.enabled) {
      interpolated_map_T_Le_Lt_[unique_ns_[i]] = T_Le_Lt;
    }
    const M3F R_Le_Lt = T_Le_Lt.rotation().matrix().cast<float>();
    const V3F t_Le_Lt = T_Le_Lt.translation().cast<float>();
    for (const size_t idx : idxs_at_unique_ns_[i]) {
      points_full_[idx].getVector3fMap() = R_Le_Lt * points_full_[idx].getVector3fMap() + t_Le_Lt;
    }
  }

  debug_msg_.t_deskew = sw.elapsedMs();
}

void Manager::preprocess(const gtsam::Key key)
{
  Stopwatch sw_preprocess;

  photometric_->preprocess(
    points_raw_, points_full_, interpolated_map_T_Le_Lt_, corrected_ts_, key);
  geometric_->preprocess(points_full_, geometric_point_idxs_, corrected_ts_);

  debug_msg_.t_preprocess_geo_photo = sw_preprocess.elapsedMs();
}

void Manager::getFactors(
  const gtsam::Values & initial_values, gtsam::NonlinearFactorGraph & new_factors)
{
  Stopwatch sw;
  logger_->debug("Getting factors");

  geometric_->getFactors(
    X(new_key_), initial_values, new_factors, geometric_eigenvectors_block_matrix_,
    geometric_degen_directions_);

  // // If none of the directions are degenerate, dont add photometric
  // if (geometric_degen_directions.norm() > 1e-6) {
  //   photometric_->getFactors(
  //     initial_values, new_factors, geometric_eigenvectors_block_matrix, geometric_degen_directions);
  // }
  M66 geometric_eigenvectors_block_matrix = M66::Identity();
  V6D geometric_degen_directions = V6D::Ones();
  photometric_->getFactors(
    initial_values, new_factors, geometric_eigenvectors_block_matrix, geometric_degen_directions);

  logger_->debug("Got factors");
  debug_msg_.t_factor_prep = sw.elapsedMs();
}

void Manager::define(
  const gtsam::NonlinearFactorGraph & new_factors, gtsam::Values & optimized_values,
  const graph::Manager::DeclarationResult result)
{
  Stopwatch sw;

  graph_manager_->define(new_factors, optimized_values, result);

  debug_msg_.t_define = sw.elapsedMs();
}

void Manager::postDefineUpdate(const gtsam::Key key, const gtsam::Values & values)
{
  Stopwatch sw;
  logger_->debug("Updating map");

  geometric_->updateMap(key, values);
  std::vector<V3D> bias_directions;
  for (size_t i = 0; i < 3; ++i) {
    if (geometric_degen_directions_[i + 3]) {
      bias_directions.push_back(
        geometric_eigenvectors_block_matrix_.bottomRightCorner<3, 3>().col(i));
    }
  }

  if (!bias_directions.size()) {
    logger_->info("Nothing degen. Selecting all directions");
    bias_directions.push_back(V3D::UnitX());
    bias_directions.push_back(V3D::UnitY());
    bias_directions.push_back(V3D::UnitZ());
  }
  photometric_->updateMap(values, bias_directions);

  debug_msg_.t_post_define_update = sw.elapsedMs();
}

void Manager::publishResults(const gtsam::Pose3 & T_W_Bk_opt)
{
  Stopwatch sw;
  logger_->debug("Publishing results");

  static int counter = 0;
  if (counter % config_.full_res_pointcloud_publish_rate_divisor == 0) {
    publishCloud(pub_points_, points_full_, config_.sensor_frame, corrected_ts_);
  }
  ++counter;

  // Write the trajectory to the trajectory logger
  // timestamp tx ty tz qx qy qz qw
  gtsam::Pose3 T_W_BkOdom = T_W_Bk_opt * config_.T_B_OdometryLoggerFrame;
  gtsam::Quaternion q = T_W_BkOdom.rotation().toQuaternion();
  trajectory_logger_->info(
    "{} {} {} {} {} {} {} {}", corrected_ts_, T_W_BkOdom.translation().x(),
    T_W_BkOdom.translation().y(), T_W_BkOdom.translation().z(), q.x(), q.y(), q.z(), q.w());

  geometric_->publishDebug();

  debug_msg_.t_publish_results = sw.elapsedMs();
}

void declare_config(ManagerConfig & config)
{
  using namespace config;
  name("Lidar Manager Config");

  field(config.logs_directory, "logs_directory", "directory_path");
  field(config.world_frame, "world_frame", "str");
  field(config.body_frame, "body_frame", "str");
  {
    NameSpace ns("lidar");
    field(config.sensor_frame, "sensor_frame", "str");
    field(config.T_B_S, "T_B_S", "gtsam::Pose3");
    field(config.T_B_OdometryLoggerFrame, "T_B_OdometryLoggerFrame", "gtsam::Pose3");
  }

  {
    NameSpace ns("lidar");
    {
      NameSpace ns("manager");
      field(config.log_level, "log_level", "trace|debug|info|warn|error|critical");
      field(config.enabled, "enabled", "bool");
      field(config.use_to_init, "use_to_init", "bool");
      field(config.initial_skip, "initial_skip", "number of pointclouds to drop at the start");
      field(config.ts_offset, "ts_offset", "s");
      field(config.range_min, "range_min", "m");
      field(config.range_max, "range_max", "m");
      field(config.intensity_min, "intensity_min", "float");
      field(config.intensity_max, "intensity_max", "float");
      field(config.create_full_res_pointcloud, "create_full_res_pointcloud", "bool");
      field(
        config.full_res_pointcloud_publish_rate_divisor, "full_res_pointcloud_publish_rate_divisor",
        "int");
      field(config.use_reflectivity_as_intensity, "use_reflectivity_as_intensity", "bool");
      field(config.scale_intensity_by_sq_range, "scale_intensity_by_sq_range", "bool");
      field(config.near_range_correction, "near_range_correction", "bool");
    }
    {
      NameSpace ns("sensor/lidar_intrinsics");
      field(config.lidar_to_sensor_transform, "lidar_to_sensor_transform", "tranformation matrix");
    }
  }

  check(config.initial_skip, GE, 0, "initial_skip");
  check(config.body_frame, NE, config.world_frame, "body_frame");
  check(
    config.sensor_frame, NE, config.world_frame,
    "sensor_frame should not be the same as world_frame");
  check(
    config.sensor_frame, NE, config.body_frame,
    "sensor_frame should not be the same as body_frame");
  check(config.range_min, GE, 0.0, "range_min");
  check(config.range_max, GT, config.range_min, "range_max");
  check(config.intensity_min, GE, 0.0, "intensity_min");
  check(config.intensity_max, GT, config.intensity_min, "intensity_max");
}

}  // namespace lidar
}  // namespace mimosa
