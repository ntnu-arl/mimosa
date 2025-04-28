// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// C++
#include <stdexcept>

// ROS
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// GTSAM
#include <gtsam/geometry/Pose3.h>

// config_utilities
#include "config_utilities/config.h"          // Enables declare_config().
#include "config_utilities/formatting/asl.h"  // Simply including this file sets a style to format output.
#include "config_utilities/logging/log_to_ros.h"  // Simply including this file sets logging to roslog.
#include "config_utilities/parsing/ros.h"         // Enable fromRos().
#include "config_utilities/printing.h"            // Enable toString()
#include "config_utilities/traits.h"              // Enables isConfig()
#include "config_utilities/types/eigen_matrix.h"  // Enable parsing and printing of Eigen::Matrix types.
#include "config_utilities/types/enum.h"          // Enable parsing and printing of enum types.
#include "config_utilities/validation.h"          // Enable isValid() and checkValid().

// mimosa
#include "mimosa/custom_yaml_types.hpp"

// spdlog
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
// clang-format off
#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
// The order of these includes is important to avoid fmt linker errors
// clang-format on

// TF2
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace mimosa
{
typedef Eigen::MatrixXd M;
// typedef Eigen::VectorXd V;

// Create handy typedefs and constants for square-size matrices
// MatrixMN, MatrixN = MatrixNN, I_NxN, and Z_NxN, for M,N=1..9
#define MAKE_MATRIX_DEFS(N)                                                         \
  using M##N = Eigen::Matrix<double, N, N>;                                         \
  using M1##N = Eigen::Matrix<double, 1, N>;                                        \
  using M2##N = Eigen::Matrix<double, 2, N>;                                        \
  using M3##N = Eigen::Matrix<double, 3, N>;                                        \
  using M4##N = Eigen::Matrix<double, 4, N>;                                        \
  using M5##N = Eigen::Matrix<double, 5, N>;                                        \
  using M6##N = Eigen::Matrix<double, 6, N>;                                        \
  using M7##N = Eigen::Matrix<double, 7, N>;                                        \
  using M8##N = Eigen::Matrix<double, 8, N>;                                        \
  using M9##N = Eigen::Matrix<double, 9, N>;                                        \
  static const Eigen::MatrixBase<M##N>::IdentityReturnType I##N = M##N::Identity(); \
  static const Eigen::MatrixBase<M##N>::ConstantReturnType Z##N = M##N::Zero();     \
  static const Eigen::MatrixBase<M1##N>::ConstantReturnType Z1##N = M1##N::Zero();  \
  static const Eigen::MatrixBase<M2##N>::ConstantReturnType Z2##N = M2##N::Zero();  \
  static const Eigen::MatrixBase<M3##N>::ConstantReturnType Z3##N = M3##N::Zero();  \
  static const Eigen::MatrixBase<M4##N>::ConstantReturnType Z4##N = M4##N::Zero();  \
  static const Eigen::MatrixBase<M5##N>::ConstantReturnType Z5##N = M5##N::Zero();  \
  static const Eigen::MatrixBase<M6##N>::ConstantReturnType Z6##N = M6##N::Zero();  \
  static const Eigen::MatrixBase<M7##N>::ConstantReturnType Z7##N = M7##N::Zero();  \
  static const Eigen::MatrixBase<M8##N>::ConstantReturnType Z8##N = M8##N::Zero();  \
  static const Eigen::MatrixBase<M9##N>::ConstantReturnType Z9##N = M9##N::Zero();

MAKE_MATRIX_DEFS(1)
MAKE_MATRIX_DEFS(2)
MAKE_MATRIX_DEFS(3)
MAKE_MATRIX_DEFS(4)
MAKE_MATRIX_DEFS(5)
MAKE_MATRIX_DEFS(6)
MAKE_MATRIX_DEFS(7)
MAKE_MATRIX_DEFS(8)
MAKE_MATRIX_DEFS(9)

typedef Eigen::VectorXd VXD;
typedef Eigen::Vector2d V2D;
typedef Eigen::Vector3d V3D;
typedef Eigen::Vector4d V4D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Matrix4d M4D;

typedef Eigen::Vector3f V3F;
typedef Eigen::MatrixXf MXF;
typedef Eigen::Matrix3f M3F;
typedef Eigen::MatrixX3f MX3F;
typedef Eigen::MatrixXd MXD;
typedef Eigen::Matrix<double, Eigen::Dynamic, 1> MX1D;
typedef Eigen::MatrixX3d MX3D;
typedef Eigen::Matrix<double, 6, 6> M6D;
typedef Eigen::Matrix<double, 6, 1> V6D;

inline std::unique_ptr<spdlog::logger> createLogger(
  const std::string & log_path, const std::string & logger_name, const std::string & level,
  const bool also_to_console = true)
{
  // Parse level
  spdlog::level::level_enum logger_level;
  if (level == "trace") {
    logger_level = spdlog::level::trace;
  } else if (level == "debug") {
    logger_level = spdlog::level::debug;
  } else if (level == "info") {
    logger_level = spdlog::level::info;
  } else if (level == "warn") {
    logger_level = spdlog::level::warn;
  } else if (level == "error") {
    logger_level = spdlog::level::err;
  } else if (level == "critical") {
    logger_level = spdlog::level::critical;
  } else {
    throw std::runtime_error("Invalid log level: " + level);
  }

  // Create logging sinks
  // Sinks are set to trace so that they will always capture anything written to them. The control is on the logger level
  std::shared_ptr<spdlog::sinks::basic_file_sink_mt> logger_file_sink_ =
    std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_path, true);
  logger_file_sink_->set_level(spdlog::level::trace);

  std::vector<spdlog::sink_ptr> sinks = {logger_file_sink_};

  if (also_to_console) {
    std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> logger_console_sink_ =
      std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    logger_console_sink_->set_level(spdlog::level::trace);
    sinks.push_back(logger_console_sink_);
  }

  // Create logger
  std::unique_ptr<spdlog::logger> logger_ =
    std::make_unique<spdlog::logger>(logger_name, sinks.begin(), sinks.end());
  logger_->set_level(logger_level);
  logger_->flush_on(logger_level);

  return logger_;
}

template <typename T>
constexpr T deg2rad(const T deg)
{
  return deg * static_cast<T>(M_PI) / static_cast<T>(180.0);
}

template <typename T>
constexpr T rad2deg(const T rad)
{
  return rad * static_cast<T>(180.0) / static_cast<T>(M_PI);
}

inline geometry_msgs::Point toGeometryMsgs(const V3D & v)
{
  geometry_msgs::Point p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}

inline void convert(const gtsam::Point3 & p, geometry_msgs::Vector3 & v)
{
  v.x = p.x();
  v.y = p.y();
  v.z = p.z();
}

inline void convert(const gtsam::Quaternion & q, geometry_msgs::Quaternion & v)
{
  v.x = q.x();
  v.y = q.y();
  v.z = q.z();
  v.w = q.w();
}

inline void convert(const gtsam::Pose3 & T_gtsam, geometry_msgs::Transform & T_gm)
{
  convert(T_gtsam.translation(), T_gm.translation);
  convert(T_gtsam.rotation().toQuaternion(), T_gm.rotation);
}

inline void convert(const V3D & v, boost::array<float, 3> & arr)
{
  arr[0] = v(0);
  arr[1] = v(1);
  arr[2] = v(2);
}

template <typename Broadcaster>
inline void broadcastTransform(
  Broadcaster & tf2_broadcaster, const gtsam::Pose3 & T_frame_child, const std::string & frame_id,
  const std::string & child_frame_id, const double ts)
{
  geometry_msgs::TransformStamped ts_frame_child;
  ts_frame_child.header.stamp.fromSec(ts);
  ts_frame_child.header.frame_id = frame_id;
  ts_frame_child.child_frame_id = child_frame_id;
  convert(T_frame_child, ts_frame_child.transform);
  tf2_broadcaster.sendTransform(ts_frame_child);
}

inline void convert(const gtsam::Pose3 & T, geometry_msgs::Pose & pose)
{
  pose.position = toGeometryMsgs(T.translation());
  convert(T.rotation().toQuaternion(), pose.orientation);
}

inline void publishPath(
  ros::Publisher & pub, nav_msgs::Path & path, const gtsam::Pose3 & T, const double ts)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = path.header.frame_id;
  pose.header.stamp.fromSec(ts);
  path.header.stamp.fromSec(ts);
  convert(T, pose.pose);
  path.poses.push_back(pose);
  pub.publish(path);
}

template <typename ExceptionType>
inline void logCriticalException(
  const std::unique_ptr<spdlog::logger> & logger, const std::string & msg)
{
  logger->critical(msg);
  throw ExceptionType(msg);
}

inline void computeLocalizability(const M33 & J_T_J, V3D & localizability, M33 & eigenvectors)
{
  Eigen::SelfAdjointEigenSolver<M33> es(J_T_J);
  localizability = es.eigenvalues().cwiseSqrt();
  eigenvectors = es.eigenvectors();
}

inline void addTriadMarker(
  const M33 & vectors, const std::string & frame_id, const double ts, const std::string & ns,
  visualization_msgs::MarkerArray & ma)
{
  for (int i = 0; i < 3; i++) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp.fromSec(ts);
    marker.ns = ns;
    marker.id = i;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.03;
    marker.scale.y = 0.05;
    marker.color.a = 1.0;
    // Color scheme is due to the colors of the lines in plotjuggler :(
    marker.color.r = i == 1;
    marker.color.g = i == 2;
    marker.color.b = i == 0;

    geometry_msgs::Point point;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);

    point = toGeometryMsgs(vectors.col(i));
    marker.points.push_back(point);

    ma.markers.push_back(marker);
  }
}

}  // namespace mimosa
