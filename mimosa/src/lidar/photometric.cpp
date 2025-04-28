// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/lidar/photometric.hpp"

namespace mimosa
{
namespace lidar
{
Photometric::Photometric(ros::NodeHandle & pnh)
: config(config::checkValid(config::fromRos<PhotometricConfig>(pnh))),
  erosion_kernel_([&]() {
    int k_size = config.patch_size + config.erosion_buffer;
    return cv::Mat::ones(k_size, k_size, CV_32FC1);
  }()),
  mask_margin_([&]() {
    cv::Mat mask = cv::Mat::zeros(config.rows, config.cols, CV_8UC1);
    mask.rowRange(config.margin_size, config.rows - config.margin_size)
      .colRange(config.margin_size, config.cols - config.margin_size)
      .setTo(1);
    return mask;
  }()),
  idx_to_u_(getIdxToPixelMap(0)),
  idx_to_v_(getIdxToPixelMap(1))
{
  // Prepare config
  logger_ = createLogger(
    config.logs_directory + "lidar_photometric.log", "lidar::Photometric", config.log_level);
  logger_->info("lidar::Photometric initialized with params:\n {}", config::toString(config, true));

  pub_img_intensity_ = pnh.advertise<sensor_msgs::Image>("lidar/photometric/img_intensity", 1);
  pub_img_new_features_ =
    pnh.advertise<sensor_msgs::Image>("lidar/photometric/img_new_features", 1);
  pub_img_tracked_keyframe_features_ =
    pnh.advertise<sensor_msgs::Image>("lidar/photometric/img_tracked_keyframe_features", 1);
  pub_features_ = pnh.advertise<sensor_msgs::PointCloud2>("lidar/photometric/features", 1);
  pub_feature_marker_ =
    pnh.advertise<visualization_msgs::Marker>("lidar/photometric/feature_marker", 1);
  pub_img_mask_ = pnh.advertise<sensor_msgs::Image>("lidar/photometric/img_mask", 1);

  pub_feature_marker_array_ = pnh.advertise<visualization_msgs::MarkerArray>(
    "lidar/photometric/feature_marker_array", 1, true);
  pub_localizability_marker_array_ = pnh.advertise<visualization_msgs::MarkerArray>(
    "lidar/photometric/localizability_marker_array", 1, true);

  pub_debug_ = pnh.advertise<mimosa_msgs::LidarPhotometricDebug>("lidar/photometric/debug", 1);

  // Check if there is a static mask to be loaded
  if (config.static_mask_path != "") {
    logger_->info("Loading static mask from path: {}", config.static_mask_path);
    static_mask_ = cv::imread(config.static_mask_path, cv::IMREAD_GRAYSCALE);
    if (static_mask_.empty()) {
      logCriticalException<std::runtime_error>(
        logger_,
        fmt::format("Static mask could not be loaded from path: {}", config.static_mask_path));
    }
    // The mask should be the same size as the image
    if (static_mask_.rows != config.rows || static_mask_.cols != config.cols) {
      logCriticalException<std::runtime_error>(
        logger_, fmt::format(
                   "Static mask size does not match image size: {}x{} vs {}x{}", static_mask_.rows,
                   static_mask_.cols, config.rows, config.cols));
    }
  } else {
    static_mask_ = cv::Mat::zeros(config.rows, config.cols, CV_8UC1);
  }
}

std::vector<int> Photometric::getIdxToPixelMap(int axis) const
{
  // Setup standard maps
  // u : column number
  // v : row number
  std::vector<int> idx_to_u = std::vector<int>(config.cols * config.rows, -1);
  std::vector<int> idx_to_v = std::vector<int>(config.cols * config.rows, -1);

  for (size_t v = 0; v < config.rows; v++) {
    for (size_t u = 0; u < config.cols; u++) {
      const int uu = (u + config.cols - config.pixel_shift_by_row[v]) % config.cols;
      const int idx = v * config.cols + (config.destagger ? uu : u);
      idx_to_u[idx] = u;
      idx_to_v[idx] = v;
    }
  }

  return axis == 0 ? idx_to_u : idx_to_v;
}

void Photometric::preprocess(
  const pcl::PointCloud<Point> & points_raw, pcl::PointCloud<Point> & points_deskewed,
  const boost::container::flat_map<uint32_t, gtsam::Pose3> & interpolated_map_T_Le_Lt,
  const double ts, const gtsam::Key key)
{
  Stopwatch sw;
  logger_->trace("Preprocess start");
  if (!config.enabled) {
    logger_->trace("Preprocess end due to not being enabled");
    return;
  }

  if (points_deskewed.size() > config.rows * config.cols) {
    logCriticalException<std::runtime_error>(
      logger_, fmt::format(
                 "Number of points exceeds the image size: {} > {} \nPerhaps check the params that "
                 "you have set for rows and cols",
                 points_deskewed.size(), config.rows * config.cols));
  }

  // Create the frame
  current_frame_ = std::make_shared<Frame>(
    config.rows, config.cols, ts, key, points_deskewed, interpolated_map_T_Le_Lt);

  cv::Mat yaw_angle_validities = cv::Mat::zeros(config.rows, config.cols, CV_8UC1);

  // Iterate over the points raw to get the yaw angles for each point
  for (size_t i = 0; i < points_raw.size(); i++) {
    const Point & p = points_raw[i];

    // Get the PBID location
    const int u = idx_to_u_[p.idx];
    const int v = idx_to_v_[p.idx];

    current_frame_->yaw_angles[v][u] = atan2(p.y, p.x);
    yaw_angle_validities.ptr<uchar>(v)[u] = 1;
  }

  // visualize(yaw_angle_validities, "raw yaw_angle_validities");
  // visualize(current_frame_->yaw_angles, "raw yaw_angles");

  for (int v = 0; v < config.rows; ++v) {
    // Collect columns that are already valid in this row
    std::vector<int> valid_cols;
    valid_cols.reserve(config.cols);

    for (int u = 0; u < config.cols; ++u) {
      if (yaw_angle_validities.ptr<uchar>(v)[u] == 1) {
        valid_cols.push_back(u);
      }
    }

    // No valid columns => entire row from +π to -π
    if (valid_cols.empty()) {
      for (int u = 0; u < config.cols; ++u) {
        float t = static_cast<float>(u) / static_cast<float>(config.cols - 1);
        current_frame_->yaw_angles[v][u] = (1.0f - t) * M_PI + t * (-M_PI);
        yaw_angle_validities.ptr<uchar>(v)[u] = 1;
      }
      continue;  // move to the next row
    }

    // Fill the region BEFORE the first valid column (if needed)
    int first_valid_col = valid_cols.front();
    if (first_valid_col > 0) {
      float yaw_first = current_frame_->yaw_angles[v][first_valid_col];
      // Interpolate from +π at col 0 to yaw_first at col first_valid_col
      for (int u = 0; u < first_valid_col; ++u) {
        float t = static_cast<float>(u) / static_cast<float>(first_valid_col);
        current_frame_->yaw_angles[v][u] = (1.0f - t) * M_PI + t * yaw_first;
        yaw_angle_validities.ptr<uchar>(v)[u] = 1;
      }
    }

    //Fill holes between consecutive valid columns
    for (size_t i = 0; i < valid_cols.size() - 1; ++i) {
      int left_col = valid_cols[i];
      int right_col = valid_cols[i + 1];

      // If adjacent, no interpolation needed
      if (right_col - left_col <= 1) continue;

      float yaw_left = current_frame_->yaw_angles[v][left_col];
      float yaw_right = current_frame_->yaw_angles[v][right_col];
      float denom = static_cast<float>(right_col - left_col);

      for (int fill_col = left_col + 1; fill_col < right_col; ++fill_col) {
        float t = static_cast<float>(fill_col - left_col) / denom;
        current_frame_->yaw_angles[v][fill_col] = yaw_left + t * (yaw_right - yaw_left);
        yaw_angle_validities.ptr<uchar>(v)[fill_col] = 1;
      }
    }

    // Fill the region AFTER the last valid column (if needed)
    int last_valid_col = valid_cols.back();
    if (last_valid_col < config.cols - 1) {
      float yaw_last = current_frame_->yaw_angles[v][last_valid_col];
      // Interpolate from yaw_last at col last_valid_col to -π at col config.cols-1
      int rightmost_col = config.cols - 1;
      int gap_size = (rightmost_col - last_valid_col);
      for (int u = last_valid_col + 1; u <= rightmost_col; ++u) {
        float t = static_cast<float>(u - last_valid_col) / static_cast<float>(gap_size);
        current_frame_->yaw_angles[v][u] = (1.0f - t) * yaw_last + t * (-M_PI);
        yaw_angle_validities.ptr<uchar>(v)[u] = 1;
      }
    }
  }

  // visualize(yaw_angle_validities, "interpolated yaw_angle_validities");
  // visualize(current_frame_->yaw_angles, "interpolated yaw_angles");

  std::vector<int> idx_to_uk(points_deskewed.size(), -1);
  std::vector<int> idx_to_vk(points_deskewed.size(), -1);

  // Fill the images
  logger_->trace("Filling images");
  // #pragma omp parallel for
  for (size_t i = 0; i < points_deskewed.size(); i++) {
    const Point & p_i_Le = points_deskewed[i];

    // Check range
    if (p_i_Le.range < config.range_min || p_i_Le.range > config.range_max) continue;

    // Fill values
    const int u = idx_to_u_[p_i_Le.idx];
    const int v = idx_to_v_[p_i_Le.idx];
    current_frame_->img_intensity.ptr<float>(v)[u] = p_i_Le.intensity;
    // Note this is the range of the skewed point
    current_frame_->img_range.ptr<float>(v)[u] = p_i_Le.range;
    current_frame_->img_mask.ptr<uchar>(v)[u] = 1;
    current_frame_->img_deskewed_cloud_idx.ptr<int>(v)[u] = i;

    V2D uv;
    if (!project(p_i_Le.getVector3fMap().cast<double>(), uv, current_frame_->yaw_angles, config))
      continue;
    idx_to_uk[i] = std::round(uv.x());
    idx_to_vk[i] = std::round(uv.y());
  }

  // Fill the proj_idx_
  logger_->trace("Filling proj_idx_");
  for (size_t i = 0; i < points_deskewed.size(); i++) {
    const int u = idx_to_uk[i];
    const int v = idx_to_vk[i];
    if (u < 0 || v < 0) continue;
    const int start_idx = vectorIndexFromRowCol(v, u, config);
    const int offset = current_frame_->proj_idx[start_idx] + 1;
    if (offset >= DUPLICATE_POINTS) continue;
    const int idx = start_idx + offset;
    current_frame_->proj_idx[idx] = i;
    current_frame_->proj_idx[start_idx] = offset;
  }

  if (config.visualize) {
    visualize(current_frame_->img_intensity, "img_intensity_");
  }

  // Scale the intensity image
  if (config.intensity_scale != 1.0) {
    current_frame_->img_intensity *= config.intensity_scale;

    if (config.visualize) {
      visualize(current_frame_->img_intensity, "img_intensity_scaled");
    }
  }

  // Apply a gamma correction
  if (config.intensity_gamma != 1.0) {
    cv::pow(current_frame_->img_intensity, config.intensity_gamma, current_frame_->img_intensity);

    if (config.visualize) {
      visualize(current_frame_->img_intensity, "img_intensity_gamma");
    }
  }

  // Remove the line artifacts due to irregular beam spacing
  if (config.remove_lines) {
    removeLines(current_frame_->img_intensity);

    if (config.visualize) {
      visualize(current_frame_->img_intensity, "img_intensity_lines_removed");
    }
  }

  // Filter the brightness to remove the effect of the distance and incidance angle
  if (config.filter_brightness) {
    filterBrightness(current_frame_->img_intensity);

    if (config.visualize) {
      visualize(current_frame_->img_intensity, "img_intensity_brightness_filtered");
    }
  }

  // Smooth the image to reduce noise
  if (config.gaussian_blur) {
    cv::GaussianBlur(
      current_frame_->img_intensity, current_frame_->img_intensity,
      cv::Size(config.gaussian_blur_size, config.gaussian_blur_size), 0);

    if (config.visualize) {
      visualize(current_frame_->img_intensity, "img_intensity_gaussian_blur");
    }
  }

  // Threshold the image
  cv::threshold(
    current_frame_->img_intensity, current_frame_->img_intensity, 255.0, 255.0, cv::THRESH_TRUNC);
  if (config.visualize) {
    visualize(current_frame_->img_intensity, "img_intensity_thresholded");
  }

  // Set the corrected intensities
  // #pragma omp parallel for
  for (size_t v = 0; v < config.rows; v++) {
    for (size_t u = 0; u < config.cols; u++) {
      const int idx = current_frame_->img_deskewed_cloud_idx.ptr<int>(v)[u];
      if (idx == -1) continue;
      points_deskewed[idx].intensity = current_frame_->img_intensity.ptr<float>(v)[u];
    }
  }

  cv::Sobel(current_frame_->img_intensity, current_frame_->img_dx, CV_32F, 1, 0, 1, 0.5);
  cv::Sobel(current_frame_->img_intensity, current_frame_->img_dy, CV_32F, 0, 1, 1, 0.5);

  createMask();

  logger_->trace("Preprocess end");
  debug_msg_.t_preprocess = sw.elapsedMs();
}

void Photometric::removeLines(cv::Mat & img)
{
  static const cv::Mat high_pass_fir = cv::Mat(config.high_pass_fir).clone();
  static const cv::Mat low_pass_fir = cv::Mat(config.low_pass_fir).clone().t();

  // Vertical high pass filter
  cv::Mat im_hpf;
  cv::filter2D(img, im_hpf, CV_32F, high_pass_fir);
  // Horizontal low pass filter
  cv::Mat lines;
  cv::filter2D(im_hpf, lines, CV_32F, low_pass_fir);
  // Remove the lines
  img -= lines;
  // Clip the negative values
  img.setTo(0, img < 0);
}

void Photometric::filterBrightness(cv::Mat & img)
{
  // Create the brightness map
  cv::Mat brightness;
  cv::blur(img, brightness, config.brightness_window_size_cv);
  brightness += 1;
  // Normalize and scale the image
  img = (140.0 * img / brightness);
}

void Photometric::createMask()
{
  // Setup the masks for the ouster connector and the frame of the robot
  // The static mask has zero for all invalid points
  // these zeros should be applied to the current_frame_ mask
  if (config.static_mask_path != "") {
    for (int v = 0; v < config.rows; v++) {
      for (int u = 0; u < config.cols; u++) {
        if (static_mask_.ptr<uchar>(v)[u] == 0) {
          current_frame_->img_mask.ptr<uchar>(v)[u] = 0;
        }
      }
    }
  }

  // Erode the mask so that there is a margin around the valid points
  cv::erode(current_frame_->img_mask, current_frame_->img_mask, erosion_kernel_);

  // Publish a mask image
  cv::Mat img_mask_copy = current_frame_->img_mask.clone();
  img_mask_copy *= 255;
  publishImage(pub_img_mask_, img_mask_copy, "mono8", config.sensor_frame, current_frame_->ts);
}

void Photometric::getFactors(
  const gtsam::Values & values, gtsam::NonlinearFactorGraph & graph,
  const M66 & eigenvectors_block_matrix, const V6D & selection)
{
  if (!config.enabled) return;
  Stopwatch sw;

  if (!world_Le_features_.size()) return;

  // Get a const ptr to the current frame
  Frame::ConstPtr const_current_frame = std::const_pointer_cast<const Frame>(current_frame_);
  M66 S = M66::Zero();
  for (size_t i = 0; i < 6; i++) {
    S(i, i) = selection(i);
  }
  photometric_factor_.reset(new PhotometricFactor(
    const_current_frame, world_Le_features_, config, eigenvectors_block_matrix, S));

  graph.add(photometric_factor_);

  debug_msg_.t_get_factors = sw.elapsedMs();
}

void Photometric::updateMap(const gtsam::Values & values, const std::vector<V3D> & bias_directions)
{
  if (!config.enabled) return;
  Stopwatch sw;

  auto const_current_frame = std::const_pointer_cast<const Frame>(current_frame_);

  if (photometric_factor_ != nullptr) {
    // Get the localizability
    V3D localizability_trans_final, localizability_rot_final;
    M33 eigenvectors_trans, eigenvectors_rot;
    photometric_factor_->getLocalizabilities(
      localizability_trans_final, localizability_rot_final, eigenvectors_trans, eigenvectors_rot);

    logger_->trace("Localizabilities computed");

    convert(localizability_trans_final, debug_msg_.localizability_trans_final);
    convert(localizability_rot_final, debug_msg_.localizability_rot_final);

    // Enforce convention of positive x for the leading eigenvector
    // This does not make a difference to the localizability
    if (eigenvectors_trans(0, 0) < 0) {
      // Flip all the eigenvectors
      eigenvectors_trans *= -1;
    }
    if (eigenvectors_rot(0, 0) < 0) {
      // Flip all the eigenvectors
      eigenvectors_rot *= -1;
    }

    // Create the marker for the axis
    visualization_msgs::MarkerArray ma;
    addTriadMarker(
      eigenvectors_trans, config.body_frame, current_frame_->ts, "LocalizabilityTrans", ma);
    addTriadMarker(
      eigenvectors_rot, config.body_frame, current_frame_->ts, "LocalizabilityRot", ma);

    pub_localizability_marker_array_.publish(ma);

    const std::vector<PhotometricFactor::RejectStatus> & statuses =
      photometric_factor_->getStatuses();
    debug_msg_.n_rejected_point_project_undistorted = 0;
    debug_msg_.n_rejected_point_range = 0;
    debug_msg_.n_rejected_point_project = 0;
    debug_msg_.n_rejected_point_mask = 0;
    debug_msg_.n_rejected_point_mask_margin = 0;
    debug_msg_.n_rejected_point_range_diff = 0;
    debug_msg_.n_rejected_point_max_error = 0;
    debug_msg_.n_correspondances = 0;
    for (size_t i = 0; i < statuses.size(); i++) {
      switch (statuses[i]) {
        case PhotometricFactor::RejectStatus::PointProjectUndistorted:
          debug_msg_.n_rejected_point_project_undistorted++;
          break;
        case PhotometricFactor::RejectStatus::PointRange:
          debug_msg_.n_rejected_point_range++;
          break;
        case PhotometricFactor::RejectStatus::PointProject:
          debug_msg_.n_rejected_point_project++;
          break;
        case PhotometricFactor::RejectStatus::PointMask:
          debug_msg_.n_rejected_point_mask++;
          break;
        case PhotometricFactor::RejectStatus::PointMaskMargin:
          debug_msg_.n_rejected_point_mask_margin++;
          break;
        case PhotometricFactor::RejectStatus::PointRangeDiff:
          debug_msg_.n_rejected_point_range_diff++;
          break;
        case PhotometricFactor::RejectStatus::MaxError:
          debug_msg_.n_rejected_point_max_error++;
          break;
        case PhotometricFactor::RejectStatus::Valid:
          debug_msg_.n_correspondances++;
          break;

        default:
          break;
      }
    }

    std::vector<size_t> invalid_indices;
    for (size_t i = 0; i < statuses.size(); i++) {
      if (statuses[i] != PhotometricFactor::RejectStatus::Valid) {
        invalid_indices.push_back(i);
      } else {
        // Update the new centers
        world_Le_features_[i].center = photometric_factor_->getFeatures().at(i).center;
        // Increase the life time of each of the tracked features by 1
        world_Le_features_[i].life_time++;

        if (world_Le_features_[i].life_time >= config.max_feature_life_time) {
          logger_->debug(
            "Feature {} has reached max life time {}", i, config.max_feature_life_time);
          invalid_indices.push_back(i);
        }
      }
    }

    // The photometric factor should never be used after this
    photometric_factor_.reset();

    // Remove all the invalid indices in reverse order
    for (auto i = invalid_indices.rbegin(); i != invalid_indices.rend(); i++) {
      world_Le_features_.erase(world_Le_features_.begin() + *i);
    }
  }

  // Use the statuses to find the features that are still being tracked
  detectFeatures(
    config.num_features_detect - world_Le_features_.size(), const_current_frame, world_Le_features_,
    values.at<gtsam::Pose3>(current_frame_->key), bias_directions);

  debug_msg_.t_update_map =
    sw.elapsedMs();  // This is done here since we are publishing the debug msg in publishFeatures

  logger_->trace("Update map end {}", values.size());
  publishFeatures(current_frame_, values, "mimosa_world", ros::Time::now().toSec());
}

void Photometric::detectFeatures(
  const int num_to_detect, Frame::ConstPtr frame, std::vector<Feature> & features,
  const gtsam::Pose3 & T_W_Be, const std::vector<V3D> & bias_directions)
{
  logger_->trace("detectFeatures start. size of feature array is {}", features.size());

  if (num_to_detect <= 0) return;

  cv::Mat detection_mask = frame->img_mask & mask_margin_;
  cv::erode(detection_mask, detection_mask, erosion_kernel_);
  for (const auto & feature : features) {
    const int u = feature.center(0);
    const int v = feature.center(1);
    cv::circle(detection_mask, cv::Point(u, v), config.nma_radius, 0, -1);
  }

  if (config.visualize) {
    visualize(detection_mask, "mask_detect");
  }
  // Compute the approximate gradient magnitude
  cv::Mat dx_abs, dy_abs, img_grad_magnitude;
  cv::convertScaleAbs(frame->img_dx, dx_abs);
  cv::convertScaleAbs(frame->img_dy, dy_abs);
  cv::addWeighted(dx_abs, 0.5, dy_abs, 0.5, 0, img_grad_magnitude);

  // Mask out the gradient image using the mask_detect
  // Sort the pixels by gradient magnitude
  std::vector<std::pair<double, cv::Point>> gradients;
  gradients.reserve(img_grad_magnitude.total());
  for (size_t v = 0; v < config.rows; v++) {
    for (size_t u = 0; u < config.cols; u++) {
      if (detection_mask.ptr<uchar>(v)[u] == 0) continue;

      if (img_grad_magnitude.ptr<uchar>(v)[u] > config.gradient_threshold) {
        gradients.emplace_back(img_grad_magnitude.ptr<uchar>(v)[u], cv::Point(u, v));
      }
    }
  }

  // Sort in descending order
  std::sort(
    gradients.begin(), gradients.end(),
    [](const std::pair<double, cv::Point> & a, const std::pair<double, cv::Point> & b) {
      return a.first > b.first;
    });

  logger_->trace("Number of gradients: {}", gradients.size());

  // Non maximal suppression
  int num_added = 0;
  std::vector<cv::Point> candidates;
  for (const auto & [_, loc] : gradients) {
    if (detection_mask.ptr<uchar>(loc.y)[loc.x] == 0) continue;
    candidates.push_back(loc);
    cv::circle(detection_mask, loc, config.nma_radius, 0, -1);
  }

  logger_->trace("Number of candidates: {}", candidates.size());

  // Get the scores of the candidates in each direction
  std::vector<std::vector<std::pair<double, int>>> scores_vec(
    bias_directions.size(),
    std::vector<std::pair<double, int>>(candidates.size(), std::make_pair(0, 0)));

  for (size_t i = 0; i < candidates.size(); i++) {
    // Approximate the gradient of the patch around the candidate
    int offset = int(config.patch_size / 2) + 1;
    cv::Rect roi_ij(
      candidates[i].x - offset, candidates[i].y - offset, config.patch_size + 2,
      config.patch_size + 2);
    cv::Mat roi = frame->img_intensity(roi_ij);
    cv::Mat eidecomp;
    cv::cornerEigenValsAndVecs(roi, eidecomp, 5, 3);
    float e1 = eidecomp.ptr<cv::Vec6f>(offset)[offset][0];
    float e2 = eidecomp.ptr<cv::Vec6f>(offset)[offset][1];

    float i_x, i_y;
    if (e1 > e2) {
      i_x = eidecomp.ptr<cv::Vec6f>(offset)[offset][2];
      i_y = eidecomp.ptr<cv::Vec6f>(offset)[offset][3];
    } else {
      i_x = eidecomp.ptr<cv::Vec6f>(offset)[offset][4];
      i_y = eidecomp.ptr<cv::Vec6f>(offset)[offset][5];
    }

    M12 dI_du;
    dI_du << i_x, i_y;

    const int point_idx = frame->img_deskewed_cloud_idx.ptr<int>(candidates[i].y)[candidates[i].x];
    if (point_idx < 0) {
      continue;
    }

    V3D p = frame->points_deskewed[point_idx].getVector3fMap().cast<double>();

    M23 du_dp;
    getProjectionJacobian(p, config, du_dp);

    // Compute the scores for each direction
    for (size_t vec_idx = 0; vec_idx < bias_directions.size(); vec_idx++) {
      scores_vec[vec_idx][i] =
        std::make_pair(fabs(dI_du * (du_dp * bias_directions[vec_idx]).normalized()), i);
    }
  }

  logger_->trace("Scores computed");

  // Sort the scored for each direction in descending order
  for (size_t vec_idx = 0; vec_idx < bias_directions.size(); vec_idx++) {
    std::sort(
      scores_vec[vec_idx].begin(), scores_vec[vec_idx].end(),
      [](const std::pair<double, int> & a, const std::pair<double, int> & b) {
        return a.first > b.first;
      });
  }

  // Select the best candidate for each direction
  std::vector<int> selected_candidate_indices;
  for (size_t i = 0; i < scores_vec[0].size(); i++) {
    for (size_t vec_idx = 0; vec_idx < bias_directions.size(); vec_idx++) {
      const int idx = scores_vec[vec_idx][i].second;
      if (
        std::find(selected_candidate_indices.begin(), selected_candidate_indices.end(), idx) ==
        selected_candidate_indices.end()) {
        selected_candidate_indices.push_back(idx);
        continue;
      }
    }
  }

  logger_->trace("Selected {} candidates", selected_candidate_indices.size());

  // Add the selected candidates to the features
  for (const auto & idx : selected_candidate_indices) {
    const cv::Point & loc = candidates[idx];

    // Create the feature
    Feature feature;
    feature.key = frame->key;
    feature.id = monotonic_feature_id_++;
    feature.life_time = 1;
    feature.center << loc.x, loc.y;

    std::vector<std::pair<int, int>> sampling_locations;
    if (config.rotate_patch_to_align_with_gradient) {
      float i_x, i_y;
      {
        // Approximate the gradient of the patch around the candidate
        int offset = int(config.patch_size / 2) + 1;
        cv::Rect roi_ij(
          loc.x - offset, loc.y - offset, config.patch_size + 2, config.patch_size + 2);
        cv::Mat roi = frame->img_intensity(roi_ij);
        cv::Mat eidecomp;
        cv::cornerEigenValsAndVecs(roi, eidecomp, 5, 3);
        float e1 = eidecomp.ptr<cv::Vec6f>(offset)[offset][0];
        float e2 = eidecomp.ptr<cv::Vec6f>(offset)[offset][1];

        if (e1 > e2) {
          i_x = eidecomp.ptr<cv::Vec6f>(offset)[offset][2];
          i_y = eidecomp.ptr<cv::Vec6f>(offset)[offset][3];
        } else {
          i_x = eidecomp.ptr<cv::Vec6f>(offset)[offset][4];
          i_y = eidecomp.ptr<cv::Vec6f>(offset)[offset][5];
        }
      }
      sampling_locations = getGradientBasedLocations(i_x, i_y, config.edgelet_patch_offsets);
    } else {
      sampling_locations = config.edgelet_patch_offsets;
    }

    // Fill in the feature data
    MX3D Le_ps(sampling_locations.size(), 3);
    for (const auto & offset : sampling_locations) {
      const int u = loc.x + offset.first;
      const int v = loc.y + offset.second;

      const Point & Le_p = frame->points_deskewed[frame->img_deskewed_cloud_idx.ptr<int>(v)[u]];
      V3D WLe_p =
        config.T_B_L.inverse() * T_W_Be * config.T_B_L * Le_p.getVector3fMap().cast<double>();
      feature.Le_ps.push_back(WLe_p);
      feature.intensities.push_back(frame->img_intensity.ptr<float>(v)[u]);
      feature.uvs.push_back(V2D(u, v));
      feature.T_Le_Lts.push_back(frame->interpolated_map_T_Le_Lt.at(Le_p.t));

      Le_ps.row(feature.Le_ps.size() - 1) = feature.Le_ps.back();
      // Check that the point when projected back into the image is not in the mask
    }

    const V3D mean = Le_ps.colwise().mean();

    MX1D ones(Le_ps.rows(), 1);
    ones.setOnes();

    const MX3D centered = Le_ps.rowwise() - mean.transpose();

    if ((centered.rowwise().norm().array() > config.max_dist_from_mean).any()) {
      continue;
    }

    const M3D cov = centered.transpose() * centered / (ones.rows() - 1);

    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    V3D normal = eigensolver.eigenvectors().col(0);

    if (((centered * normal).array().abs() > config.max_dist_from_plane).any()) {
      continue;
    }

    // Check that the normal is pointing towards the camera
    if (normal.dot(mean) / mean.norm() > 0) {
      normal *= -1;
    }
    feature.normal = normal;

    VXD intensities(feature.intensities.size());
    for (int i = 0; i < feature.intensities.size(); i++) {
      intensities(i) = feature.intensities[i];
    }

    getPsi(intensities, feature.mean_intensity, feature.sigma_intensity, feature.psi_intensities);

    features.push_back(feature);
    cv::circle(detection_mask, loc, config.nma_radius, 0, -1);

    num_added++;
    if (num_added >= num_to_detect) break;
  }

  logger_->trace("detectFeatures end. size of feature array is now = {}", features.size());
}

void Photometric::publishFeatures(
  Frame::ConstPtr frame, const gtsam::Values & values, const std::string & world_frame,
  const double ts)
{
  logger_->trace("publishFeatures start");

  pcl::PointCloud<Point> cloud;

  auto addPointsToCloud = [&](const std::vector<Feature> & features, const bool map_frame) {
    for (const auto & feature : features) {
      const gtsam::Pose3 T_W_Be =
        map_frame ? gtsam::Pose3::Identity() : values.at<gtsam::Pose3>(feature.key);
      const gtsam::Pose3 T_W_Le = T_W_Be * config.T_B_L;
      for (size_t i = 0; i < feature.Le_ps.size(); i++) {
        Point p;
        p.getVector3fMap() = (T_W_Le * feature.Le_ps[i]).cast<float>();
        p.intensity = feature.intensities[i];
        cloud.push_back(p);
      }
    }
  };

  addPointsToCloud(world_Le_features_, true);
  // addPointsToCloud(last_frame_features_, false);

  publishCloud(pub_features_, cloud, world_frame, ts);

  cv::Mat img_intensity_u8;
  frame->img_intensity.convertTo(img_intensity_u8, CV_8UC1);

  publishImage(pub_img_intensity_, img_intensity_u8, "mono8", world_frame, ts);

  if (pub_img_new_features_.getNumSubscribers()) {
    cv::Mat img_new_features;
    // drawFeatures(img_intensity_u8, last_frame_features_, img_new_features);
    publishImage(pub_img_new_features_, img_new_features, "bgr8", world_frame, ts);
  }

  if (pub_img_tracked_keyframe_features_.getNumSubscribers()) {
    cv::Mat img_tracked_keyframe_features;
    drawFeatures(img_intensity_u8, world_Le_features_, img_tracked_keyframe_features);
    publishImage(
      pub_img_tracked_keyframe_features_, img_tracked_keyframe_features, "bgr8", world_frame, ts);
  }

  if (pub_feature_marker_.getNumSubscribers()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame;
    marker.header.stamp.fromSec(ts);
    marker.ns = "features";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.01;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    int center_idx = config.patch_size * config.patch_size / 2;
    // for (const auto & feature : last_frame_features_) {
    //   const gtsam::Pose3 & T_W_Be = values.at<gtsam::Pose3>(feature.key);
    //   const V3D & W_p = T_W_Be * config.T_B_L * feature.Le_ps[center_idx];
    //   marker.points.push_back(toGeometryMsgs(T_W_Be.translation()));
    //   marker.points.push_back(toGeometryMsgs(W_p));
    // }

    pub_feature_marker_.publish(marker);
  }
  logger_->trace("publishFeatures end");

  // Publish the debug message
  debug_msg_.header.stamp.fromSec(ts);
  pub_debug_.publish(debug_msg_);
}

}  // namespace lidar
}  // namespace mimosa
