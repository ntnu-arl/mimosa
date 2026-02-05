// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

namespace mimosa
{

using gtsam::symbol_shorthand::B;  // IMU Bias
using gtsam::symbol_shorthand::G;  // Gravity
using gtsam::symbol_shorthand::V;  // Velocities
using gtsam::symbol_shorthand::X;  // Pose3 (R,t)

class State
{
private:
  gtsam::Key key_;
  double ts_;
  gtsam::NavState nav_state_;
  gtsam::imuBias::ConstantBias imu_bias_;
  gtsam::Unit3 gravity_;

public:
  State() : key_(0), ts_(0) {}

  gtsam::Key key() const { return key_; }
  double ts() const { return ts_; }
  const auto & navState() const { return nav_state_; }
  const auto & imuBias() const { return imu_bias_; }
  const auto & gravity() const { return gravity_; }

  void update(
    const gtsam::Key key, const double ts, const gtsam::NavState & nav_state,
    const gtsam::imuBias::ConstantBias & imu_bias, const gtsam::Unit3 & gravity)
  {
    key_ = key;
    ts_ = ts;
    nav_state_ = nav_state;
    imu_bias_ = imu_bias;
    gravity_ = gravity;
  }
};

}  // namespace mimosa
