// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <pcl/point_types.h>

// This is kept here since it is at the top level
#define PCL_NO_PRECOMPILE

namespace mimosa
{
namespace lidar
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;  // 16 bytes
  float intensity;  // 4 bytes
  uint32_t t;       // 4 bytes // Time in nanosecs since the beginning of the scan
  uint32_t idx;     // 4 bytes // Index in the original pointcloud
  float range;      // 4 bytes
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Point() = default;

  Point(float x, float y, float z, float intensity, uint32_t t, uint32_t idx, float range)
  {
    this->x = x;
    this->y = y;
    this->z = z;
    this->intensity = intensity;
    this->t = t;
    this->idx = idx;
    this->range = range;
  }
};

// XYZIRT
struct EIGEN_ALIGN16 PointOuster
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;  // Time in nanosecs since the beginning of the scan
  uint16_t reflectivity;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// XYZIRT
struct EIGEN_ALIGN16 PointOusterR8
{
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;  // Time in nanosecs since the beginning of the scan
  uint16_t reflectivity;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// XYZIRT
struct EIGEN_ALIGN16 PointHesai
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;  // Global time (unix timestamp)
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 PointLivox
{
  PCL_ADD_POINT4D;
  float intensity;   // the value is reflectivity, 0.0~255.0
  uint8_t tag;       // livox tag
  uint8_t line;      // laser number in lidar
  double timestamp;  // Global time (unix timestamp) in nano seconds
};

struct EIGEN_ALIGN16 PointLivoxFromCustom2
{
  float x;
  float y;
  float z;
  uint32_t t;       // Time in nanosecs since the beginning of the scan
  float intensity;  // the value is reflectivity, 0.0~255.0
  uint8_t tag;      // livox tag
  uint8_t line;     // laser number in lidar
};

// XYZIRT
struct EIGEN_ALIGN16 PointVelodyne
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// XYZIRT
struct EIGEN_ALIGN16 PointVelodyneAnybotics
{
  PCL_ADD_POINT4D;
  float intensity;
  float ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// XYZIRT
struct EIGEN_ALIGN16 PointRslidar
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring;
  double timestamp;  // Global time (unix timestamp) in seconds
};
}  // namespace lidar
}  // namespace mimosa

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::lidar::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                          std::uint32_t, t, t)(std::uint32_t, idx, idx)(float, range, range))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::lidar::PointOuster,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t, t)(
    std::uint16_t, reflectivity, reflectivity)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::lidar::PointOusterR8,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t, t)(
    std::uint16_t, reflectivity, reflectivity)(std::uint8_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::lidar::PointHesai, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                               double, timestamp, timestamp)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::lidar::PointLivox,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint8_t, tag, tag)(
    std::uint8_t, line, line)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::lidar::PointLivoxFromCustom2,
  (float, x, x)(float, y, y)(float, z, z)(std::uint32_t, t, t)(float, intensity, intensity)(
    std::uint8_t, tag, tag)(std::uint8_t, line, line))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::lidar::PointVelodyne,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(
    float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::lidar::PointVelodyneAnybotics,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, ring, ring)(
    float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::lidar::PointRslidar, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                 std::uint16_t, ring, ring)(double, timestamp, timestamp))
