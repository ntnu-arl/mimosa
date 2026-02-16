// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "mimosa/radar/utils.hpp"

namespace mimosa
{
namespace radar
{
class DopplerFactor
: public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>
{
  gtsam::Point3 point_measurement_;    // point in radar frame
  double doppler_measurement_;         // doppler velocity of point
  gtsam::Pose3 pose_R_B_;              // pose of radar in B
  gtsam::Vector3 angular_velocity_B_;  // angvel from IMU during radar exposure
public:
  typedef NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias> Base;

  DopplerFactor(
    const gtsam::Point3 & point, const double doppler, const gtsam::Pose3 & pose_R_B,
    const gtsam::Vector3 & angular_velocity_B, const gtsam::Key k0, const gtsam::Key k1,
    const gtsam::Key k2, const gtsam::SharedNoiseModel & model)
  : Base(model, k0, k1, k2),
    point_measurement_(point),
    doppler_measurement_(doppler),
    pose_R_B_(pose_R_B),
    angular_velocity_B_(angular_velocity_B)
  {
  }

  virtual ~DopplerFactor() {}

  // Evaluate error h(x)-z and optionally derivatives
  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose_B_W, const gtsam::Vector3 & linear_velocity_W,
    const gtsam::imuBias::ConstantBias & imu_bias_B,
    boost::optional<gtsam::Matrix &> H0 = boost::none,
    boost::optional<gtsam::Matrix &> H1 = boost::none,
    boost::optional<gtsam::Matrix &> H2 = boost::none) const
  {
    const gtsam::Point3 point_hat = point_measurement_.normalized();  // normalize point
    const gtsam::Rot3 rot_R_B = pose_R_B_.rotation();                 // R from {R} to {B}
    const gtsam::Point3 l_R_B = pose_R_B_.translation();  // translation of {R} expressed in {B}
    const gtsam::Rot3 rot_B_W = pose_B_W.rotation();      // R from {B} to {W}

    // calculate influence of angular velocity on linear velocity through l_R_B
    const gtsam::Vector3 linear_velocity_from_angular_B =
      (angular_velocity_B_ - imu_bias_B.gyroscope()).cross(l_R_B);
    const gtsam::Vector3 linear_velocity_R =
      rot_R_B.transpose() *
      (rot_B_W.transpose() * linear_velocity_W + linear_velocity_from_angular_B);

    // residual formulated as h(x) - z
    const double doppler_estimate = -point_hat.dot(linear_velocity_R);
    const gtsam::Vector residual =
      (gtsam::Vector(1) << (doppler_estimate - doppler_measurement_)).finished();

    // df/dtau
    if (H0) {
      H0->resize(1, 6);

      (*H0).leftCols(3) = -point_hat.transpose() * rot_R_B.transpose() *
                          (rot_B_W.transpose() * gtsam::skewSymmetric(linear_velocity_W) *
                           rot_B_W.matrix());        // rotation
      (*H0).rightCols(3) = gtsam::Matrix13::Zero();  // translation
    }

    // df/dv
    if (H1) {
      H1->resize(1, 3);

      *H1 = -point_hat.transpose() * rot_R_B.transpose() * rot_B_W.transpose();
    }

    // df/dBias
    if (H2) {
      H2->resize(1, 6);

      (*H2).leftCols(3) = gtsam::Matrix13::Zero();  // accelerometer
      (*H2).rightCols(3) =
        -point_hat.transpose() * rot_R_B.transpose() * gtsam::skewSymmetric(l_R_B);  // gyroscope
    }

    return residual;
  }
};

// Unary factor which linearizes to a hessian
class DopplerHessianFactor : public gtsam::NonlinearFactor
{
public:
  enum Status
  {
    Static = 0,
    Dynamic
  };

  const std::vector<Status> & getStatus() const { return target_status_; }
  const TargetVector & getStatic() const { return static_targets_; }
  const TargetVector & getDynamic() const { return dynamic_targets_; }

private:
  /* mutable */ TargetVector targets_;  // targets in radar frame
  gtsam::Pose3 pose_R_B_;               // pose of radar in B
  gtsam::Vector3 angular_velocity_B_;   // angvel from IMU during radar exposure;

  double noise_sigma_;        // in m/s
  double huber_threshold_;    // in std deviations
  double outlier_threshold_;  // in std deviations

  mutable std::vector<Status> target_status_;  // classification of targets as static or non-static
  mutable TargetVector static_targets_;
  mutable TargetVector dynamic_targets_;

  mutable std::shared_ptr<gtsam::GaussianFactor> gaussian_factor_ = nullptr;

  mutable size_t num_active_factors_;

  typedef gtsam::NonlinearFactor Base;

public:
  DopplerHessianFactor(
    const TargetVector & targets, const gtsam::Pose3 & pose_R_B,
    const gtsam::Vector3 & angular_velocity_B, const gtsam::Key key0, const gtsam::Key key1,
    const gtsam::Key key2, const double noise_sigma, const double huber_threshold,
    const double outlier_threshold)
  : Base(std::vector<gtsam::Key>{key0, key1, key2}),
    targets_(targets),
    pose_R_B_(pose_R_B),
    angular_velocity_B_(angular_velocity_B),
    noise_sigma_(noise_sigma),
    huber_threshold_(huber_threshold),
    outlier_threshold_(outlier_threshold)
  {
    target_status_.resize(targets_.size());
  }

  ~DopplerHessianFactor() override {}

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new DopplerHessianFactor(*this)));
  }

  size_t dim() const override
  {
    return 15;  // The Hessian is 15x15
  }

  void print(
    const std::string & /*s*/ = "",
    const gtsam::KeyFormatter & keyFormatter = gtsam::DefaultKeyFormatter) const override
  {
    std::cout << "DopplerHessianFactor(" << keyFormatter(keys()[0]) << ", "
              << keyFormatter(keys()[1]) << ", " << keyFormatter(keys()[2]) << ")\n"
              << "  Targets: " << targets_.size() << "\n"
              << "  Noise Sigma: " << noise_sigma_ << "\n"
              << "  Huber Threshold: " << huber_threshold_ << "\n"
              << "  Outlier Threshold: " << outlier_threshold_ << "\n"
              << "  Pose R_B: " << pose_R_B_ << "\n"
              << "  Angular Velocity B: " << angular_velocity_B_.transpose() << "\n";
  }

  double error([[maybe_unused]] const gtsam::Values & c) const override
  {
    std::cout << "\033[1;31mCalling Error on Key: \033[0m\n" << gdkf(keys()[0]) << "\n";
    return 0.0;
  }

  std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values & c) const override
  {
    const gtsam::Rot3 rot_R_B = pose_R_B_.rotation();     // R from {R} to {B}
    const gtsam::Point3 l_R_B = pose_R_B_.translation();  // translation of {R} expressed in {B}
    const gtsam::Pose3 pose_B_W = c.at<gtsam::Pose3>(keys()[0]);  // pose of {B} wrt {W}
    const gtsam::Rot3 rot_B_W = pose_B_W.rotation();              // R from {B} to {W}
    const gtsam::Vector3 linear_velocity_W =
      c.at<gtsam::Vector3>(keys()[1]);  // linear velocity in {W}
    const gtsam::imuBias::ConstantBias imu_bias_B =
      c.at<gtsam::imuBias::ConstantBias>(keys()[2]);  // imu bias

    // calculate influence of angular velocity on linear velocity through l_R_B
    const gtsam::Vector3 linear_velocity_from_angular_B =
      (angular_velocity_B_ - imu_bias_B.gyroscope()).cross(l_R_B);
    const gtsam::Vector3 linear_velocity_R =
      rot_R_B.transpose() *
      (rot_B_W.transpose() * linear_velocity_W + linear_velocity_from_angular_B);

    gtsam::Matrix66 G11 = gtsam::Matrix66::Zero();
    gtsam::Matrix63 G12 = gtsam::Matrix63::Zero();
    gtsam::Matrix66 G13 = gtsam::Matrix66::Zero();
    gtsam::Matrix33 G22 = gtsam::Matrix33::Zero();
    gtsam::Matrix36 G23 = gtsam::Matrix36::Zero();
    gtsam::Matrix66 G33 = gtsam::Matrix66::Zero();
    gtsam::Vector6 g1 = gtsam::Vector6::Zero();
    gtsam::Vector3 g2 = gtsam::Vector3::Zero();
    gtsam::Vector6 g3 = gtsam::Vector6::Zero();
    double f = 0.0;

    int i = -1;
    static_targets_.clear();
    static_targets_.reserve(targets_.size());
    dynamic_targets_.clear();
    dynamic_targets_.reserve(targets_.size());
    for (const TargetData & t : targets_) {
      i++;

      const gtsam::Point3 bearing = gtsam::Point3(t.x, t.y, t.z) / t.range;
      const double doppler = t.radial_speed;

      // Compute the error
      const double e = -bearing.dot(linear_velocity_R) - doppler;

      // Compute the Jacobians
      gtsam::Matrix16 J1;
      J1.leftCols(3) = -bearing.transpose() * rot_R_B.transpose() *
                       (rot_B_W.transpose() * gtsam::skewSymmetric(linear_velocity_W) *
                        rot_B_W.matrix());        // rotation;
      J1.rightCols(3) = gtsam::Matrix13::Zero();  // translation;
      gtsam::Matrix13 J2 =
        -bearing.transpose() * rot_R_B.transpose() * rot_B_W.transpose();  // velocity
      gtsam::Matrix16 J3;
      J3.leftCols(3) = gtsam::Matrix13::Zero();  // accelerometer;
      J3.rightCols(3) =
        -bearing.transpose() * rot_R_B.transpose() * gtsam::skewSymmetric(l_R_B);  // gyroscope

      // Set whitened errors
      gtsam::Matrix J1_whitened = J1 / noise_sigma_;
      gtsam::Matrix J2_whitened = J2 / noise_sigma_;
      gtsam::Matrix J3_whitened = J3 / noise_sigma_;
      double e_whitened = e / noise_sigma_;

      // Apply Robust cost
      const double c = 2.3849;
      const double weight = std::sqrt(1 / (1 + std::pow(e_whitened / c, 2)));

      J1_whitened *= weight;
      J2_whitened *= weight;
      J3_whitened *= weight;
      e_whitened *= weight;

      G11 += J1_whitened.transpose() * J1_whitened;
      G12 += J1_whitened.transpose() * J2_whitened;
      G13 += J1_whitened.transpose() * J3_whitened;
      G22 += J2_whitened.transpose() * J2_whitened;
      G23 += J2_whitened.transpose() * J3_whitened;
      G33 += J3_whitened.transpose() * J3_whitened;
      g1 += -J1_whitened.transpose() * e_whitened;
      g2 += -J2_whitened.transpose() * e_whitened;
      g3 += -J3_whitened.transpose() * e_whitened;
      f += e_whitened * e_whitened;
    }

    // std::cout << static_targets_.size() << " static and " << dynamic_targets_.size()
    //           << " dynamic targets\n";

    // Create the factor
    return std::make_shared<gtsam::HessianFactor>(
      keys()[0], keys()[1], keys()[2], G11, G12, G13, g1, G22, G23, g2, G33, g3, f);
  }
};
}  // namespace radar
}  // namespace mimosa
