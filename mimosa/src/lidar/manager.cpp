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
: SensorManagerBase<ManagerConfig, sensor_msgs::PointCloud2>(
    config::checkValid(config::fromRos<ManagerConfig>(pnh)), imu_manager, graph_manager, "lidar"),
  z_offset_(-config_.lidar_to_sensor_transform[11] / 1000.0),
  range_min_sq_(config_.range_min * config_.range_min),
  range_max_sq_(config_.range_max * config_.range_max)
{
  geometric_ = std::make_unique<Geometric>(pnh);
  photometric_ = std::make_unique<Photometric>(pnh);

  // Checks that span across configs
  if (!config_.create_full_res_pointcloud && photometric_->config.enabled) {
    logCriticalException<std::runtime_error>(
      logger_, "Photometric being enabled requires create_full_res_pointcloud to be true.");
  }

  debug_msg_.initialized = initialized_;

  pub_points_ = pnh.advertise<sensor_msgs::PointCloud2>("lidar/manager/points_full_res", 1);
  pub_debug_ = pnh.advertise<mimosa_msgs::LidarManagerDebug>("lidar/manager/debug", 1);

  // Setup trajectory logger
  trajectory_logger_ = createLogger(
    config_.base.logs_directory + "lidar_manager_odometry.log", "lidar::Manager::odometry", "trace", false);
  trajectory_logger_->set_pattern("%v");

  subscribeIfEnabled(pnh);
}

void Manager::callback(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  Stopwatch sw;
  if (!passesCommonValidations(msg)) {
    return;
  }

  // Preprocess the pointcloud
  static const PType point_type = decodePointType(msg->fields);
  switch (point_type) {
    case PType::Ouster:
      prepareInput<PointOuster>(msg);
      break;
    case PType::OusterOdyssey:
      prepareInput<PointOusterOdyssey>(msg);
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
    case PType::VelodyneAnybotics:
      prepareInput<PointVelodyneAnybotics>(msg);
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
  graph::Manager::DeclarationResult dr =
    graph_manager_->declare(corrected_ts_, new_key_, config_.base.use_to_init);

  if (!handleDeclarationResult(dr)) {
    return;
  }
  debug_msg_.t_declare = sw_declare.elapsedMs();
  logger_->debug("Declaration done for ts: {} assigned key: {}", corrected_ts_, gdkf(new_key_));

  if (imu_preintegrator_ == nullptr) {
    imu_preintegrator_ =
      std::make_unique<gtsam::PreintegratedImuMeasurements>(imu_manager_->getPreintegratorParams());
  }

  deskewPoints();

  preprocess(X(new_key_));

  static bool first = true;
  gtsam::Pose3 T_W_Bk_opt;
  static gtsam::Values opt_values = graph_manager_->getCurrentOptimizedValues();
  if (first) {
    first = false;
    broadcastStaticTransform(corrected_ts_);

    initialized_ = true;
    debug_msg_.initialized = initialized_;

    logger_->info("Initialized");

    T_W_Bk_opt = graph_manager_->getPoseAt(new_key_);
    opt_values.insert_or_assign(X(new_key_), T_W_Bk_opt);
  } else {
    // Get the new factors
    gtsam::Values initial_values = opt_values;
    initial_values.insert_or_assign(X(new_key_), graph_manager_->getPoseAt(new_key_));
    gtsam::NonlinearFactorGraph new_factors;
    getFactors(initial_values, new_factors);

    define(new_factors, opt_values, dr);

    // const gtsam::NonlinearFactorGraph factors = graph_manager_->getFactors();
    // photometric_->visualizeTracks(
    //   factors, opt_values, config_.map_frame, corrected_ts_, X(new_key_));

    T_W_Bk_opt = opt_values.at<gtsam::Pose3>(X(new_key_));
  }

  postDefineUpdate(X(new_key_), opt_values);

  publishResults(T_W_Bk_opt);

  debug_msg_.header.stamp.fromSec(corrected_ts_);
  debug_msg_.t_full = sw.elapsedMs();
  logger_->debug("Callback complete for key {}. Full processing time: {} ms", gdkf(new_key_), debug_msg_.t_full);
  pub_debug_.publish(debug_msg_);
}

template <typename PointT>
void Manager::prepareInput(const sensor_msgs::PointCloud2::ConstPtr & msg)
{
  Stopwatch sw;

  // Convert the pointcloud to pcl
  static pcl::PointCloud<PointT> msg_cloud;
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
  points_full_.reserve(
    config_.create_full_res_pointcloud ? msg_cloud.size()
                                       : msg_cloud.size() / geometric_->config.point_skip_divisor);
  geometric_point_idxs_.reserve(msg_cloud.size() / geometric_->config.point_skip_divisor);
  ns_idx_pairs_.reserve(msg_cloud.size());
  unique_ns_.reserve(msg_cloud.size());
  idxs_at_unique_ns_.reserve(msg_cloud.size());
  debug_msg_.t_clear_reserve = sw.tickMs();

  if constexpr (std::is_same<PointT, PointRslidar>::value || 
                std::is_same<PointT, PointVelodyneAnybotics>::value) {
    // Transposing the pointcloud is required for RSAiry
    // since the remaining code assumes the height corresponds to the number of rings
    // and the width corresponds to the number of points per ring.
    // The pointcloud of the VelodyneAnybotics lidar also seems to be transposed
    Stopwatch sw_transpose;
    if (config_.transpose_pointcloud) {
      pcl::PointCloud<PointT> msg_cloud_transposed;
      msg_cloud_transposed.width = msg_cloud.height;
      msg_cloud_transposed.height = msg_cloud.width;
      msg_cloud_transposed.resize(msg_cloud_transposed.width * msg_cloud_transposed.height);

      for (size_t i = 0; i < msg_cloud.size(); ++i) {
        const size_t current_row = i / msg_cloud.width;
        const size_t current_col = i % msg_cloud.width;

        const size_t new_row = current_col;
        const size_t new_col = current_row;
        msg_cloud_transposed[new_row * msg_cloud_transposed.width + new_col] = msg_cloud[i];
      }
      msg_cloud = std::move(msg_cloud_transposed);
    }
    debug_msg_.t_transpose_pointcloud = sw_transpose.elapsedMs();
  }

  if constexpr (
    !std::is_same<PointT, PointLivox>::value &&
    !std::is_same<PointT, PointLivoxFromCustom2>::value && 
    !std::is_same<PointT, PointOusterOdyssey>::value) {
    Stopwatch sw_organize;
    if (config_.organize_pointcloud_by_ring && msg_cloud.height == 1) {
      // The main loop in the pointcloud skips some points in each ring since the resolution in a ring is typically higher as compared to the number of rings. It assumes that the pointcloud is provided in a row major order. If the input pointcloud is not in row major order, it should be organized as such so that the point skipping logic does not discard too many useful points.
      // This was needed for the Hesai JT128 but could also be useful for other lidars.
      constexpr uint32_t num_rings = 128; // This is set to 128 since that is the largest number of channels in common lidars. AFAIK only velodyne alpha prime has 256 channels.
      const size_t num_points = msg_cloud.size();
      // Single pass to count points per ring
      std::array<size_t, num_rings> ring_counts{};
      for (const auto & point : msg_cloud) {
        ++ring_counts[point.ring];
      }

      // Compute starting indices for each ring
      std::array<size_t, num_rings> ring_offsets{};
      size_t offset = 0;
      for (uint32_t i = 0; i < num_rings; ++i) {
        ring_offsets[i] = offset;
        offset += ring_counts[i];
      }

      // Place points directly into final positions
      pcl::PointCloud<PointT> msg_cloud_organized;
      msg_cloud_organized.resize(num_points);

      std::array<size_t, num_rings> ring_cursors = ring_offsets;
      for (const auto & point : msg_cloud) {
        msg_cloud_organized[ring_cursors[point.ring]++] = point;
      }

      msg_cloud = std::move(msg_cloud_organized);
    }
    debug_msg_.t_organize_by_ring = sw_organize.elapsedMs();
  }

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
      if (std::isnan(pin.tag) || !((pin.tag & 0x30) == 0x10 || (pin.tag & 0x30) == 0x00)) {
        continue;
      }
    }

    // Intensity filter
    float intensity = 0.0f;
    if constexpr (std::is_same<PointT, PointOusterOdyssey>::value) {
      if (std::isnan(pin.reflectivity) || pin.reflectivity < config_.intensity_min ||
          pin.reflectivity > config_.intensity_max)
        continue;
      intensity = static_cast<float>(pin.reflectivity);
    } else {
      if (
        std::isnan(pin.intensity) || pin.intensity < config_.intensity_min ||
        pin.intensity > config_.intensity_max)
        continue;
      intensity = pin.intensity;
    }

    // Range filter
    const float range_sq = pin.x * pin.x + pin.y * pin.y + pin.z * pin.z;
    if (range_sq < range_min_sq_ || range_sq > range_max_sq_) continue;

    // Decode time to be nanoseconds since the header timestamp
    uint32_t t_ns;
    if constexpr (
      std::is_same<PointT, PointOuster>::value || std::is_same<PointT, PointOusterOdyssey>::value || std::is_same<PointT, PointOusterR8>::value) {
      t_ns = pin.t;
    } else if constexpr (std::is_same<PointT, PointHesai>::value) {
      t_ns = (pin.timestamp - header_ts_) * 1e9;
    } else if constexpr (std::is_same<PointT, PointLivox>::value) {
      t_ns = pin.timestamp - header_ts_ * 1e9;
    } else if constexpr (std::is_same<PointT, PointLivoxFromCustom2>::value) {
      t_ns = pin.t;
    } else if constexpr (std::is_same<PointT, PointVelodyne>::value || 
                         std::is_same<PointT, PointVelodyneAnybotics>::value) {
      t_ns = pin.time * 1e9;
    } else if constexpr (std::is_same<PointT, PointRslidar>::value) {
      t_ns = (pin.timestamp - header_ts_) * 1e9;
    } else {
      throw std::runtime_error("Time decoding not implemented for this point type");
    }

    if (t_ns > config_.ns_max) {
      continue;
    }

    last_point_ns = std::max(last_point_ns, t_ns);

    points_full_.points.emplace_back(
      pin.x, pin.y, pin.z + z_offset_, intensity, t_ns, i, std::sqrt(range_sq));
    const size_t new_idx = points_full_.size() - 1;
    ns_idx_pairs_.emplace_back(t_ns, new_idx);

    // Point Skip filter
    if (i % geometric_->config.point_skip_divisor != 0) continue;

    // Ring filter - does not make sense in a Livox pointcloud
    // VelodyneAnybotics points also seem to have unreliable ring numbers
    if constexpr (
      !std::is_same<PointT, PointLivox>::value &&
      !std::is_same<PointT, PointLivoxFromCustom2>::value &&
      !std::is_same<PointT, PointVelodyneAnybotics>::value &&
      !std::is_same<PointT, PointOusterOdyssey>::value) {
      if (std::isnan(pin.ring))
      {
        logger_->warn("Point ring number is NaN. Skipping the point but check your input data");
        continue;
      }
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
    "Filtering done, header ts: {}, corrected ts: {}, points_full_size: {}, duration: {}",
    header_ts_, corrected_ts_, points_full_.size(), corrected_ts_ - header_ts_);

  // Save a copy of the raw points. This is needed by photometric
  if (photometric_->config.enabled) {
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

  static int not_initialized_count = 0;
  if (!initialized_) {
    ++not_initialized_count;
    if (not_initialized_count != 1) {
      logger_->error("not_initialized_count: {} but should be 1.", not_initialized_count);
    }
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
  imu_manager_->getInterpolatedMeasurements(state.ts(), corrected_ts_, imu_measurements, true);

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
  const gtsam::Pose3 T_Le_W = config_.base.T_B_S.inverse() * propagated_state_.pose().inverse();

  for (size_t i = 0; i < unique_ns_.size(); ++i) {
    const gtsam::Pose3 T_Le_Lt = T_Le_W * T_W_Bts_at_unique_ns[i] * config_.base.T_B_S;

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
  const graph::Manager::DeclarationResult dr)
{
  Stopwatch sw;
  logger_->debug(
    "Defining with {} new factors for key: {}", new_factors.size(),
    gdkf(new_key_));

  graph_manager_->define(new_factors, optimized_values, dr);

  logger_->debug("Define done for key : {}", gdkf(new_key_));
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
    publishCloud(pub_points_, points_full_, config_.base.sensor_frame, corrected_ts_);
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

  // Declare common sensor manager config fields
  declare_sensor_manager_config_base(config.base, "lidar");

  // Declare lidar-specific fields
  {
    NameSpace ns("lidar");
    field(config.T_B_OdometryLoggerFrame, "T_B_OdometryLoggerFrame", "gtsam::Pose3");
    {
      NameSpace ns("manager");
      field(config.transpose_pointcloud, "transpose_pointcloud", "bool");
      field(config.organize_pointcloud_by_ring, "organize_pointcloud_by_ring", "bool");
      field(config.range_min, "range_min", "m");
      field(config.range_max, "range_max", "m");
      field(config.intensity_min, "intensity_min", "float");
      field(config.intensity_max, "intensity_max", "float");
      field(config.ns_max, "ns_max", "float");
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

  // Lidar-specific validation
  check(config.range_min, GE, 0.0, "range_min");
  check(config.range_max, GT, config.range_min, "range_max");
  check(config.intensity_min, GE, 0.0, "intensity_min");
  check(config.intensity_max, GT, config.intensity_min, "intensity_max");
}

}  // namespace lidar
}  // namespace mimosa
