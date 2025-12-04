// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// mimosa
#include "mimosa/radar/point.hpp"
#include "mimosa/utils.hpp"

namespace mimosa
{
namespace radar
{
struct TargetData
{
  double x;
  double y;
  double z;
  double range;
  double azimuth;
  double elevation;
  double radial_speed;
  double intensity;

  TargetData(
    double x, double y, double z, double range, double azimuth, double elevation,
    double radial_speed, double intensity)
  : x(x),
    y(y),
    z(z),
    range(range),
    azimuth(azimuth),
    elevation(elevation),
    radial_speed(radial_speed),
    intensity(intensity)
  {
  }
};
typedef std::vector<TargetData> TargetVector;

enum class PType
{
  Unknown,
  Rio,
  mmWave,
  mmWaveDopplerResidual
};

inline PType decodePointType(const std::vector<sensor_msgs::PointField> & fields)
{
  if (fieldsMatch(fields, getFieldsFromPointType<rioPoint>())) {
    return PType::Rio;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<mmWavePoint>())) {
    return PType::mmWave;
  }

  if (fieldsMatch(fields, getFieldsFromPointType<mmWaveDopplerResidualPoint>())) {
    return PType::mmWaveDopplerResidual;
  }

  return PType::Unknown;
}
}  // namespace radar
}  // namespace mimosa
