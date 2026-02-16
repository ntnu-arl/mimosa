// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "mimosa/utils.hpp"

namespace mimosa
{
namespace odometry
{
/** \brief Calculate Doptimality metric for checking external odometry
   * @param covMat: covariance matrix of external odometry
   * @return dOptVal: return calculated D-optimality metric
   */
inline double calcDoptimality(const MXD & covMat)
{
  return std::exp(std::log(std::pow(covMat.determinant(), (1.0 / covMat.rows()))));
}

inline void convert(const boost::array<double, 36> & in, MXD & out)
{
  out = MXD::Zero(6, 6);
  for (size_t i = 0; i < 6; ++i) {
    for (size_t j = 0; j < 6; ++j) {
      out(i, j) = in[i * 6 + j];
    }
  }
}

}  // namespace odometry
}  // namespace mimosa
