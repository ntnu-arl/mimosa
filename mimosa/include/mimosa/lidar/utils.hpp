// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa/lidar/point.hpp"
#include "mimosa/utils.hpp"

// ROS
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// cv_bridge
#include <cv_bridge/cv_bridge.h>

// C++
#include <Eigen/Core>
#include <algorithm>
#include <boost/functional/hash/hash.hpp>
#include <numeric>

namespace mimosa
{
namespace lidar
{
enum class PType
{
  Unknown = 0,
  Ouster,
  OusterOdyssey,
  OusterR8,
  Hesai,
  Livox,
  LivoxFromCustom2,
  Velodyne,
  VelodyneAnybotics,
  Rslidar
};

inline PType decodePointType(const std::vector<sensor_msgs::PointField> & fields)
{
  if (fieldsMatch(fields, getFieldsFromPointType<PointOuster>())) {
    return PType::Ouster;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<PointOusterOdyssey>())) {
    return PType::OusterOdyssey;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<PointOusterR8>())) {
    return PType::OusterR8;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<PointHesai>())) {
    return PType::Hesai;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<PointLivox>())) {
    return PType::Livox;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<PointLivoxFromCustom2>())) {
    return PType::LivoxFromCustom2;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<PointVelodyne>())) {
    return PType::Velodyne;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<PointVelodyneAnybotics>())) {
    return PType::VelodyneAnybotics;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<PointRslidar>())) {
    return PType::Rslidar;
  }

  return PType::Unknown;
}

inline void publishImage(
  ros::Publisher & pub, const cv::Mat & img, const std::string & encoding,
  const std::string & frame_id, const double ts)
{
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), encoding, img).toImageMsg();
  msg->header.frame_id = frame_id;
  msg->header.stamp.fromSec(ts);
  pub.publish(msg);
}

inline Point toPcl(const gtsam::Point3 & p)
{
  Point pcl_point;
  pcl_point.x = p.x();
  pcl_point.y = p.y();
  pcl_point.z = p.z();
  return pcl_point;
}

inline void applyTransform(const gtsam::Pose3 & T_B_A, const Point p_A, Point & p_B)
{
  p_B = p_A;
  p_B.getVector3fMap() = (T_B_A * p_A.getVector3fMap().cast<double>()).cast<float>();
}

inline void applyTransform(
  const gtsam::Pose3 & T_B_A, const pcl::PointCloud<Point> & cloud_A,
  pcl::PointCloud<Point> & cloud_B)
{
  // Copy all fields
  cloud_B.header = cloud_A.header;
  cloud_B.points.resize(cloud_A.size());
  cloud_B.width = cloud_A.width;
  cloud_B.height = cloud_A.height;
  cloud_B.is_dense = cloud_A.is_dense;
  cloud_B.sensor_origin_ = cloud_A.sensor_origin_;
  cloud_B.sensor_orientation_ = cloud_A.sensor_orientation_;

  for (size_t i = 0; i < cloud_A.size(); i++) {
    applyTransform(T_B_A, cloud_A[i], cloud_B[i]);
  }
}

template <typename PointT>
inline void toPcl(const sensor_msgs::PointCloud2 & msg, pcl::PointCloud<PointT> & cloud)
{
  pcl_conversions::toPCL(msg.header, cloud.header);
  cloud.width = msg.width;
  cloud.height = msg.height;
  cloud.is_dense = msg.is_dense == 1;

  pcl::MsgFieldMap field_map;
  pcl::createMapping<PointT>(msg.fields, field_map);

  // Copy point data
  std::uint32_t num_points = msg.width * msg.height;
  cloud.points.resize(num_points);
  std::uint8_t * cloud_data = reinterpret_cast<std::uint8_t *>(&cloud.points[0]);

  // Check if we can copy adjacent points in a single memcpy.  We can do so if there
  // is exactly one field to copy and it is the same size as the source and destination
  // point types.
  if (
    field_map.size() == 1 && field_map[0].serialized_offset == 0 &&
    field_map[0].struct_offset == 0 && field_map[0].size == msg.point_step &&
    field_map[0].size == sizeof(PointT)) {
    std::uint32_t cloud_row_step = static_cast<std::uint32_t>(sizeof(PointT) * cloud.width);
    const std::uint8_t * msg_data = &msg.data[0];
    // Should usually be able to copy all rows at once
    if (msg.row_step == cloud_row_step) {
      memcpy(cloud_data, msg_data, msg.data.size());
    } else {
      for (std::uint32_t i = 0; i < msg.height;
           ++i, cloud_data += cloud_row_step, msg_data += msg.row_step)
        memcpy(cloud_data, msg_data, cloud_row_step);
    }

  } else {
    // If not, memcpy each group of contiguous fields separately
    for (std::uint32_t row = 0; row < msg.height; ++row) {
      const std::uint8_t * row_data = &msg.data[row * msg.row_step];
      for (std::uint32_t col = 0; col < msg.width; ++col) {
        const std::uint8_t * msg_data = row_data + col * msg.point_step;
        for (const pcl::detail::FieldMapping & mapping : field_map) {
          memcpy(
            cloud_data + mapping.struct_offset, msg_data + mapping.serialized_offset, mapping.size);
        }
        cloud_data += sizeof(PointT);
      }
    }
  }
}

inline void errorBetween(
  const gtsam::Pose3 & T1, const gtsam::Pose3 & T2, float & err_rot, float & err_trans)
{
  gtsam::Pose3 T1_T2 = T1.between(T2);
  err_trans = T1_T2.translation().norm();
  err_rot = T1_T2.rotation().axisAngle().second;
}

inline bool getProjectionMatrix(
  const V3D localizability, const double thresh, const M33 & eigenvectors, M33 & P,
  V3D & degenerate_axes)
{
  // Check if degenerate before doing more work
  if ((localizability.array() > thresh).all()) {
    // Nothing is degenerate
    P = I3;
    degenerate_axes = Z31;
    return false;
  }

  P = Z33;
  degenerate_axes = V3D::Ones();
  for (int i = 0; i < 3; i++) {
    if (localizability(i) > thresh) {
      // Add the eigenvector to the projection matrix
      P += eigenvectors.col(i) * eigenvectors.col(i).transpose();
      degenerate_axes(i) = 0;
    }
  }
  return true;
}

/// @brief Fast floor (https://stackoverflow.com/questions/824118/why-is-floor-so-slow).
/// @param pt  Double vector
/// @return    Floored int vector
inline Eigen::Array4i fast_floor(const Eigen::Array4d & pt)
{
  const Eigen::Array4i ncoord = pt.cast<int>();
  return ncoord - (pt < ncoord.cast<double>()).cast<int>();
}

/**
 * @brief Spatial hashing function
 *        Teschner et al., "Optimized Spatial Hashing for Collision Detection of Deformable Objects", VMV2003
 */
class XORVector3iHash
{
public:
  size_t operator()(const Eigen::Vector3i & x) const
  {
    const size_t p1 = 9132043225175502913;
    const size_t p2 = 7277549399757405689;
    const size_t p3 = 6673468629021231217;
    return static_cast<size_t>((x[0] * p1) ^ (x[1] * p2) ^ (x[2] * p3));
  }
};

struct FlatContainerMinimal
{
public:
  struct Setting
  {
    double min_sq_dist_in_cell = 0.1 * 0.1;  ///< Minimum squared distance between points in a cell.
    size_t max_num_points_in_cell = 20;      ///< Maximum number of points in a cell.
  };

  FlatContainerMinimal(const Setting & setting)
  {
    points.reserve(setting.max_num_points_in_cell);
    indices.reserve(setting.max_num_points_in_cell);
  }
  size_t size() const { return points.size(); }

  /// @brief Add a point to the container.
  /// @param setting The downsampling settings for the voxel.
  /// @param point_pos The 3D position of the point to potentially add.
  /// @return True if the point was added, false otherwise.
  bool add(const Setting & setting, const Eigen::Vector3d & point_pos, const size_t index)
  {
    if (points.size() >= setting.max_num_points_in_cell) {
      return false;  // Voxel is full
    }

    // Check minimum distance constraint
    for (const auto & existing_pt : points) {
      if ((existing_pt - point_pos).squaredNorm() < setting.min_sq_dist_in_cell) {
        return false;  // Too close to an existing point in this voxel
      }
    }

    // Add the point if constraints are met
    points.push_back(point_pos);
    indices.push_back(index);

    return true;
  }

  void clear()
  {
    points.clear();
    indices.clear();
  }

  /// @brief Get the points stored in this container.
  const std::vector<Eigen::Vector3d> & get_points() const { return points; }
  /// @brief Get the indices of the points in the original cloud.
  const std::vector<size_t> & get_indices() const { return indices; }

private:
  std::vector<Eigen::Vector3d> points;  ///< Points
  std::vector<size_t> indices;          ///< Indices of the points in the original cloud
};
}  // namespace lidar
}  // namespace mimosa
