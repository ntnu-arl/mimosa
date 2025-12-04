// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <pcl/point_types.h>
#include <sensor_msgs/PointField.h>

namespace mimosa
{
namespace radar
{
struct rioPoint
{
  PCL_ADD_POINT4D;      // position [m]
  float snr_db;         // CFAR cell to side noise ratio [dB]
  float noise_db;       // CFAR noise level of the side of the detected cell [dB]
  float v_doppler_mps;  // Doppler speed [m/s]
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct mmWavePoint
{
  PCL_ADD_POINT4D;  // position [m]
  float intensity;  // RCS?
  float velocity;   // Doppler speed [m/s]
};

struct mmWaveDopplerResidualPoint
{
  PCL_ADD_POINT4D;  // position [m]
  float intensity;  // RCS?
  float velocity;   // Doppler speed [m/s]
  float doppler_residual;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace radar
}  // namespace mimosa

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::radar::rioPoint, (float, x, x)(float, y, y)(float, z, z)(float, snr_db, snr_db)(
                             float, noise_db, noise_db)(float, v_doppler_mps, v_doppler_mps))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::radar::mmWavePoint,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity))

POINT_CLOUD_REGISTER_POINT_STRUCT(
  mimosa::radar::mmWaveDopplerResidualPoint,
  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, velocity, velocity)(
    float, doppler_residual, doppler_residual))
