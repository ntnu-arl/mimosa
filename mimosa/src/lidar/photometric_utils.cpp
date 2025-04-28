// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/lidar/photometric_utils.hpp"

namespace mimosa
{
namespace lidar
{
void getPsi(const VXD & i_bar, double & mean, double & sigma, VXD & psi)
{
  mean = i_bar.mean();
  sigma = (i_bar.array() - mean).matrix().norm();
  psi = (i_bar.array() - mean) / sigma;
}

void getPsiJacobian(const double sigma, const VXD & psi, MXD & H)
{
  const size_t m = psi.size();

  H = ((Eigen::MatrixXd::Identity(m, m) - psi * psi.transpose()) / sigma) *
      (Eigen::MatrixXd::Identity(m, m) - Eigen::MatrixXd::Ones(m, m) / m);
}

bool inFOV(const V2D & uv, const int rows, const int cols)
{
  return uv.x() >= 0 && uv.x() <= cols - 1 && uv.y() >= 0 && uv.y() <= rows - 1;
}

bool inFOV(const V2D & uv, const PhotometricConfig & config)
{
  return inFOV(uv, config.rows, config.cols);
}

bool project(const V3D & p, V2D & uv, const PhotometricConfig & config)
{
  const double L = sqrt(p.x() * p.x() + p.y() * p.y()) - config.beam_offset_m;
  const double R = sqrt(L * L + p.z() * p.z());
  const double phi = atan2(p.y(), p.x());
  const double theta = asin(p.z() / R);

  uv(0) = config.fx * phi + config.cx;

  if (uv(0) < 0 || uv(0) >= config.cols) {
    std::string msg = fmt::format(
      "Invalid x coordinate: {} for phi: {}, L: {}, R: {}, fx: {}, cx: {}", uv(0), phi, L, R,
      config.fx, config.cx);
    std::cout << "Critical: " << msg << "\n";
    throw std::runtime_error(msg);
    return false;
  }

  // The altitude angles are in degrees and descending order
  // For OS-0, the first value is 45.0 and the last value is -45.0
  if (
    theta > DEG2RAD(*config.beam_altitude_angles.begin()) ||
    theta < DEG2RAD(*config.beam_altitude_angles.rbegin()))
    return false;

  auto greater =
    (std::upper_bound(
       config.beam_altitude_angles.rbegin(), config.beam_altitude_angles.rend(), RAD2DEG(theta)) +
     1)
      .base();

  auto smaller = greater + 1;

  // Because we have already checked the bounds, the iterator will never be at the end

  // Interpolate the y coordinate for subpixel accuracy
  uv.y() = std::distance(config.beam_altitude_angles.begin(), greater);
  uv.y() += (*greater - RAD2DEG(theta)) / (*greater - *smaller);

  return inFOV(uv, config);
}

bool project(
  const V3D & p, V2D & uv, const std::vector<std::vector<float>> & yaw_angles,
  const PhotometricConfig & config, const bool print)
{
  const double L = sqrt(p.x() * p.x() + p.y() * p.y()) - config.beam_offset_m;
  const double R = sqrt(L * L + p.z() * p.z());
  const double phi = atan2(p.y(), p.x());
  const double theta = asin(p.z() / R);

  uv(0) = config.fx * phi + config.cx;

  if (uv(0) < 0 || uv(0) >= config.cols) {
    std::string msg = fmt::format(
      "Invalid x coordinate: {} for phi: {}, L: {}, R: {}, fx: {}, cx: {}", uv(0), phi, L, R,
      config.fx, config.cx);
    std::cout << "Critical: " << msg << "\n";
    throw std::runtime_error(msg);
    return false;
  }

  // Safety margin to avoid interpolation errors
  if (uv(0) < 5 || uv(0) > config.cols - 5) return false;

  // The altitude angles are in degrees and descending order
  // For OS-0, the first value is 45.0 and the last value is -45.0
  if (
    theta > DEG2RAD(*config.beam_altitude_angles.begin()) ||
    theta < DEG2RAD(*config.beam_altitude_angles.rbegin()))
    return false;

  auto greater =
    (std::upper_bound(
       config.beam_altitude_angles.rbegin(), config.beam_altitude_angles.rend(), RAD2DEG(theta)) +
     1)
      .base();

  auto smaller = greater + 1;

  // Because we have already checked the bounds, the iterator will never be at the end

  // Interpolate the y coordinate for subpixel accuracy
  uv.y() = std::distance(config.beam_altitude_angles.begin(), greater);
  uv.y() += (*greater - RAD2DEG(theta)) / (*greater - *smaller);

  // Using the uv(0) as a guess find the exact pixel in the corresponding row of the yaw angles
  int approx_y = int(std::round(uv.y()));

  // Do a binary search to find either the value that exactly matches or the two values that contain the search value
  // Note that the yaw angles are in descending order
  auto it_left = yaw_angles[approx_y].begin() + int(uv.x()) - 5;
  auto it_right = yaw_angles[approx_y].begin() + int(uv.x()) + 5;

  while (std::distance(it_left, it_right) > 1) {
    auto mid = it_left + std::distance(it_left, it_right) / 2;
    if (*mid == phi) {
      uv.x() = std::distance(yaw_angles[approx_y].begin(), mid);
      return inFOV(uv, config);
    } else if (*mid < phi) {
      it_right = mid;
    } else {
      it_left = mid;
    }
  }

  // If the value is not found, interpolate between the two closest values
  double new_x = std::distance(yaw_angles[approx_y].begin(), it_left);
  new_x += (*it_left - phi) / (*it_left - *it_right);

  auto great = it_left;
  auto small = it_right;

  // auto great = (std::upper_bound(yaw_angles[approx_y].rbegin(), yaw_angles[approx_y].rend(), phi)).base();
  // auto small = great + 1;
  // double new_x = std::distance(yaw_angles[approx_y].begin(), great);
  // new_x += (*great - phi) / (*great - *small);

  //  Print the difference in the pixels
  if (print) {
    std::cout << "Angle: " << RAD2DEG(phi) << " greater: " << RAD2DEG(*great)
              << " idx: " << std::distance(yaw_angles[approx_y].begin(), great)
              << " smaller = " << RAD2DEG(*small)
              << " idx: " << std::distance(yaw_angles[approx_y].begin(), small)
              << " calculated idx: " << new_x << " original x: " << uv.x() << std::endl;
    std::cout << "Angle at great - 1: " << RAD2DEG(*(great - 1))
              << " idx: " << std::distance(yaw_angles[approx_y].begin(), great - 1) << std::endl;
  }

  float old_x = uv.x();
  uv.x() = new_x;

  if (!inFOV(uv, config)) {
    // std::cout << "project: uv not in fov: " << uv.transpose() << std::endl;

    // std::cout << "Angle: " << RAD2DEG(phi) << " greater: " << RAD2DEG(*great)
    //           << " idx: " << std::distance(yaw_angles[approx_y].begin(), great)
    //           << " smaller = " << RAD2DEG(*small)
    //           << " idx: " << std::distance(yaw_angles[approx_y].begin(), small)
    //           << " calculated idx: " << new_x << " original x: " << old_x << std::endl;
    // std::cout << "Angle at great - 1: " << RAD2DEG(*(great - 1))
    //           << " idx: " << std::distance(yaw_angles[approx_y].begin(), great - 1) << std::endl;

    return false;
  }
  return true;
}

void getProjectionJacobian(const V3D & Li_p, const PhotometricConfig & config, M23 & H)
{
  const double rxy = Li_p.head<2>().norm();
  const double L = rxy - config.beam_offset_m;
  const double R2 = L * L + Li_p.z() * Li_p.z();
  const double irxy = 1.0 / rxy;
  const double irxy2 = irxy * irxy;
  const double fx_irxy2 = config.fx * irxy2;

  H << -fx_irxy2 * Li_p.y(), fx_irxy2 * Li_p.x(), 0,
    -config.fy * Li_p.x() * Li_p.z() / ((L + config.beam_offset_m) * R2),
    -config.fy * Li_p.y() * Li_p.z() / ((L + config.beam_offset_m) * R2), config.fy * L / R2;
}

int vectorIndexFromRowCol(const int row, const int col, const PhotometricConfig & config)
{
  return (row * config.cols + col) * DUPLICATE_POINTS;
}

bool projectUndistorted(
  const V3D & Le_p, V3D & Li_p, V2D & Li_uv, int & distortion_idx, const bool round,
  const std::vector<int> & proj_idx, const pcl::PointCloud<Point> & points_deskewed,
  const boost::container::flat_map<uint32_t, gtsam::Pose3> & interpolated_map_T_Le_Lt_,
  const PhotometricConfig & config)
{
  // Project the point into the image at the end of the scan
  V2D Lk_uv;
  if (!project(Le_p, Lk_uv, config)) return false;

  if (round) {
    Lk_uv(0) = std::round(Lk_uv(0));
    Lk_uv(1) = std::round(Lk_uv(1));
  }

  distortion_idx = -1;
  size_t row = Lk_uv(1);
  size_t col = Lk_uv(0);

  int idx = vectorIndexFromRowCol(row, col, config);
  // Check if there are any points from the undistorted pointcloud that project to the column in this pixel
  if (proj_idx[idx] == 0) {
    // Any points in the same column would be looking in this direction at the same time
    // Therefore, we can search for the points that project in the same column
    row = 0;
    for (; row < config.rows; row++) {
      idx = vectorIndexFromRowCol(row, col, config);
      if (proj_idx[idx] > 0) break;
    }
    if (row >= config.rows) return false;
  }

  // If there are multiple points that project to the same pixel, select the one that is closest to the feature point
  if (proj_idx[idx] > 1) {
    float min_sq_distance = std::numeric_limits<float>::max();
    for (int i = 1; i <= proj_idx[idx]; i++) {
      const int j = proj_idx[idx + i];
      const V3D p_cand = points_deskewed[j].getVector3fMap().cast<double>();
      const float sq_distance = (Le_p - p_cand).squaredNorm();
      if (sq_distance < min_sq_distance) {
        min_sq_distance = sq_distance;
        distortion_idx = j;
      }
    }
  } else {
    distortion_idx = proj_idx[idx + 1];
  }

  if (distortion_idx < 0) return false;

  // Find the distortion transformation for the point
  gtsam::Pose3 T_Le_Lt;
  try {
    T_Le_Lt = interpolated_map_T_Le_Lt_.at(points_deskewed[distortion_idx].t);
  } catch (const std::out_of_range & e) {
    std::cerr << "in project undistorted looked up time " << points_deskewed[distortion_idx].t
              << e.what() << '\n';
    std::cout << "times in interpolated_map_T_Le_Lt_: ";

    // sort the times and print them
    std::vector<uint32_t> times;
    for (auto const & [key, val] : interpolated_map_T_Le_Lt_) {
      times.push_back(key);
    }
    std::sort(times.begin(), times.end());
    for (auto const & time : times) {
      std::cout << time << " ";
    }
    std::cout << "\n";

    throw e;
  }

  // Distort the point
  Li_p = T_Le_Lt.inverse() * Le_p;

  // Project the point into the intensity image
  if (!project(Li_p, Li_uv, config)) return false;

  return true;
}

bool projectUndistorted(
  const V3D & Le_p, V3D & Li_p, V2D & Li_uv, gtsam::Pose3 & T_Le_Lt, const bool round,
  const std::vector<int> & proj_idx, const pcl::PointCloud<Point> & points_deskewed,
  const boost::container::flat_map<uint32_t, gtsam::Pose3> & interpolated_map_T_Le_Lt_,
  const std::vector<std::vector<float>> & yaw_angles, const PhotometricConfig & config)
{
  // Project the point into the image at the end of the scan
  V2D Lk_uv;
  if (!project(Le_p, Lk_uv, yaw_angles, config)) return false;

  if (round) {
    Lk_uv(0) = std::round(Lk_uv(0));
    Lk_uv(1) = std::round(Lk_uv(1));
  }

  int distortion_idx = -1;
  size_t row = Lk_uv(1);
  size_t col = Lk_uv(0);

  int idx = vectorIndexFromRowCol(row, col, config);
  // Check if there are any points from the undistorted pointcloud that project to the column in this pixel
  if (proj_idx[idx] == 0) {
    // Any points in the same column would be looking in this direction at the same time
    // Therefore, we can search for the points that project in the same column
    row = 0;
    for (; row < config.rows; row++) {
      idx = vectorIndexFromRowCol(row, col, config);
      if (proj_idx[idx] > 0) break;
    }
    if (row >= config.rows) return false;
  }

  // If there are multiple points that project to the same pixel, select the one that is closest to the feature point
  if (proj_idx[idx] > 1) {
    float min_sq_distance = std::numeric_limits<float>::max();
    for (int i = 1; i <= proj_idx[idx]; i++) {
      const int j = proj_idx[idx + i];
      const V3D p_cand = points_deskewed[j].getVector3fMap().cast<double>();
      const float sq_distance = (Le_p - p_cand).squaredNorm();
      if (sq_distance < min_sq_distance) {
        min_sq_distance = sq_distance;
        distortion_idx = j;
      }
    }
  } else {
    distortion_idx = proj_idx[idx + 1];
  }

  if (distortion_idx < 0) return false;

  // Find the distortion transformation for the point
  try {
    T_Le_Lt = interpolated_map_T_Le_Lt_.at(points_deskewed[distortion_idx].t);
  } catch (const std::out_of_range & e) {
    std::cerr << "in project undistorted looked up time " << points_deskewed[distortion_idx].t
              << e.what() << '\n';
    std::cout << "times in interpolated_map_T_Le_Lt_: ";

    // sort the times and print them
    std::vector<uint32_t> times;
    for (auto const & [key, val] : interpolated_map_T_Le_Lt_) {
      times.push_back(key);
    }
    std::sort(times.begin(), times.end());
    for (auto const & time : times) {
      std::cout << time << " ";
    }
    std::cout << "\n";

    throw e;
  }

  // Distort the point
  Li_p = T_Le_Lt.inverse() * Le_p;

  // Project the point into the intensity image
  if (!project(Li_p, Li_uv, yaw_angles, config)) return false;

  return true;
}

double getSubPixelValue(const cv::Mat & img, const V2D loc_uv)
{
  if (!inFOV(loc_uv, img.rows, img.cols)) {
    throw std::runtime_error(
      fmt::format("Location {} is not in the field of view of the image", loc_uv.transpose()));
  }

  // Bilinear Interpolation
  const double & x = loc_uv.x();
  const double & y = loc_uv.y();
  const int x0 = std::floor(x);
  const int x1 = x0 + 1;
  const int y0 = std::floor(y);
  const int y1 = y0 + 1;

  const double dx = x - x0;
  const double dy = y - y0;

  return (1 - dx) * (1 - dy) * img.ptr<float>(y0)[x0] + dx * (1 - dy) * img.ptr<float>(y0)[x1] +
         (1 - dx) * dy * img.ptr<float>(y1)[x0] + dx * dy * img.ptr<float>(y1)[x1];
}

void drawFeatures(
  const cv::Mat & img_gray_u8, const std::vector<Feature> & features, cv::Mat & img_with_features,
  const std::vector<double> & print_vals)
{
  cv::cvtColor(img_gray_u8, img_with_features, CV_GRAY2RGB);

  cv::Mat high_res_image;
  const int scale_factor = 10;

  cv::resize(
    img_with_features, high_res_image, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);

  // Draw features on the high res image
  for (const auto & feature : features) {
    cv::Point scaled_point(feature.center(0) * scale_factor, feature.center(1) * scale_factor);
    cv::circle(
      high_res_image, scaled_point, 5 * scale_factor,
      feature.life_time == 1 ? cv::Scalar(0, 255, 0) : cv::Scalar(255, 0, 0), cv::LINE_AA);
  }
  cv::resize(high_res_image, img_with_features, img_with_features.size(), 0, 0, cv::INTER_LINEAR);

  int index = 0;
  for (const auto & feature : features) {
    // Write the feature number
    cv::putText(
      img_with_features, std::to_string(print_vals.size() ? print_vals[index] : feature.id),
      cv::Point(feature.center(0), feature.center(1)), cv::FONT_HERSHEY_SIMPLEX, 0.3,
      cv::Scalar(0, 0, 255), 1);
    index++;
  }
}

void visualize(const cv::Mat & image, const std::string & window_name)
{
  // Get the min and max values
  double min_val, max_val;
  cv::minMaxLoc(image, &min_val, &max_val);

  cv::Mat normalized_image;
  cv::normalize(image, normalized_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  // Write the min and max values on the top left corner
  cv::putText(
    normalized_image, fmt::format("min: {:.2f} max: {:.2f}", min_val, max_val), cv::Point(0, 10),
    cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(255), 1);

  cv::imshow(window_name, normalized_image);
  cv::waitKey(1);
}

void visualize(const std::vector<std::vector<float>> & v, const std::string & window_name)
{
  // Convert the 2d vector to a cv::Mat
  cv::Mat image(v.size(), v[0].size(), CV_32F);
  for (size_t i = 0; i < v.size(); i++) {
    for (size_t j = 0; j < v[i].size(); j++) {
      image.at<float>(i, j) = v[i][j];
    }
  }

  visualize(image, window_name);
}

std::pair<int, int> snapPoint(
  const std::pair<double, double> & p, std::set<std::pair<int, int>> & used)
{
  int grid_x = static_cast<int>(std::round(p.first));
  int grid_y = static_cast<int>(std::round(p.second));
  std::pair<int, int> candidate = {grid_x, grid_y};

  if (used.find(candidate) == used.end()) {
    used.insert(candidate);
    return candidate;
  }

  double best_distance = std::numeric_limits<double>::max();
  std::pair<int, int> best = candidate;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      std::pair<int, int> candidate_alt = {grid_x + dx, grid_y + dy};
      if (used.find(candidate_alt) == used.end()) {
        double diff_x = candidate_alt.first - p.first;
        double diff_y = candidate_alt.second - p.second;
        double dist = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        if (dist < best_distance) {
          best_distance = dist;
          best = candidate_alt;
        }
      }
    }
  }
  used.insert(best);
  return best;
}

std::vector<std::pair<int, int>> getGradientBasedLocations(
  const float grad_x, const float grad_y, const std::vector<std::pair<int, int>> & original_pattern)
{
  // Normalize the gradient.
  double norm = std::sqrt(grad_x * grad_x + grad_y * grad_y) + 1e-6;
  // Compute the edge normal and tangent.
  double normal_x = -grad_y / norm;
  double normal_y = grad_x / norm;
  double tangent_x = grad_x / norm;
  double tangent_y = grad_y / norm;

  // Rotate each point using the rotation matrix T = [normal, tangent].
  // For a point (x, y), the rotated coordinates are:
  //   r_x = normal_x * x + tangent_x * y
  //   r_y = normal_y * x + tangent_y * y
  std::vector<std::pair<double, double>> continuous;
  for (const auto & p : original_pattern) {
    double x = static_cast<double>(p.first);
    double y = static_cast<double>(p.second);
    double r_x = normal_x * x + tangent_x * y;
    double r_y = normal_y * x + tangent_y * y;
    continuous.push_back({r_x, r_y});
  }

  // Snap the continuous coordinates to grid points with collision resolution.
  std::vector<std::pair<int, int>> snapped;
  std::set<std::pair<int, int>> used;
  for (const auto & p : continuous) {
    std::pair<int, int> gridPoint = snapPoint(p, used);
    snapped.push_back(gridPoint);
  }

  return snapped;
}

}  // namespace lidar
}  // namespace mimosa
