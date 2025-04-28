// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// OpenMP
#include <omp.h>

// GTSAM
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

// mimosa
#include "mimosa/lidar/photometric_utils.hpp"

namespace mimosa
{
namespace lidar
{
class PhotometricFactor : public gtsam::NonlinearFactor
{
  // This is a factor constraining one pose to an implicit pose or between two poses
  // If it is a unary factor, then the key_b is T_W_B and its cloud is registered to the world frame
  // - In this case the features exist in the LiDAR_World frame and are projected into the intensity image.
  // - The LiDAR_World frame is simply T_B_L
  // If it is a binary factor, then the key_b is T_W_Be_b and features from key_a with pose T_W_Be_a are projected into the intensity image at the b frame
  // The extracted features are deskewed and in the LiDAR frame corresponding to the a frame

public:
  using Ptr = std::shared_ptr<PhotometricFactor>;

  enum class RejectStatus
  {
    Unprocessed = 0,
    PointProjectUndistorted,
    PointRange,
    PointProject,
    PointMask,
    PointMaskMargin,
    PointRangeDiff,
    MaxError,
    Valid
  };

  const std::vector<RejectStatus> & getStatuses() const { return statuses_; }

  const std::vector<Feature> & getFeatures() const { return a_features_; }

  void getLocalizabilities(
    V3D & trans_final, V3D & rot_final, M33 & eigenvectors_trans, M33 & eigenvectors_rot)
  {
    trans_final = localizability_trans_final_;
    rot_final = localizability_rot_final_;
    eigenvectors_trans = localizability_eigenvectors_trans_;
    eigenvectors_rot = localizability_eigenvectors_rot_;
  }

private:
  const bool is_binary_;
  Frame::ConstPtr b_frame_;
  mutable std::vector<Feature> a_features_;  // Features that were extracted from the a frame

  mutable std::vector<RejectStatus> statuses_;

  // QOL Typedefs
  typedef gtsam::NonlinearFactor Base;
  typedef PhotometricFactor This;
  typedef std::shared_ptr<This> shared_ptr;

  const PhotometricConfig config_;
  const M6D VSVt;

  // Localizabilities
  // Note all of these are whitened and weighted
  mutable V3D
    localizability_trans_final_;  // Localizability of translation computed from the final JtJ
  mutable V3D localizability_rot_final_;  // Localizability of rotation computed from the final JtJ
  mutable M33 localizability_eigenvectors_trans_ = M33::Identity();
  mutable M33 localizability_eigenvectors_rot_ = M33::Identity();

public:
  PhotometricFactor(
    Frame::ConstPtr b_frame, const gtsam::Key key_a, const std::vector<Feature> & a_features,
    const PhotometricConfig & config)
  : Base(std::vector<gtsam::Key>{b_frame->key, key_a}),
    is_binary_(true),
    b_frame_(b_frame),
    a_features_(a_features),
    config_(config)
  {
    if (!a_features_.size()) {
      throw std::runtime_error("No features in a_features");
    }

    statuses_.resize(a_features_.size(), RejectStatus::Unprocessed);
  }

  PhotometricFactor(
    Frame::ConstPtr b_frame, const std::vector<Feature> & a_features,
    const PhotometricConfig & config, const M6D & V = I6, const M6D & S = I6)
  : Base(std::vector<gtsam::Key>{b_frame->key}),
    is_binary_(false),
    b_frame_(b_frame),
    a_features_(a_features),
    config_(config),
    VSVt(V * S * V.transpose())
  {
    if (!a_features_.size()) {
      throw std::runtime_error("No features in a_features");
    }

    statuses_.resize(a_features_.size(), RejectStatus::Unprocessed);
  }

  ~PhotometricFactor() override {}

  gtsam::NonlinearFactor::shared_ptr clone() const override
  {
    return std::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new This(*this)));
  }

  size_t dim() const override { return 6; }

  double error(const gtsam::Values & c) const override
  {
    // This function is ignored since it only serves to increase computational cost
    std::cout << "\033[1;31mCalling Error on Key: \033[0m\n"
              << gtsam::DefaultKeyFormatter(keys()[0])
              << "with value: " << c.at<gtsam::Pose3>(keys()[0]) << "\n";
    return 0.0;
  }

  std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values & c) const override
  {
    // Iterate over all the features in the a features vector. These should be projected
    // into the b frame and compared to the b image. The error is the Least squares NCC
    // between the feature patch and the image intensities at the projected locations

    // std::cout << "Linearizing photometric with key: " << gtsam::DefaultKeyFormatter(keys()[0])
    //           << "\n";

    const gtsam::Pose3 & T_W_Be_b = c.at<gtsam::Pose3>(keys()[0]);
    const gtsam::Pose3 T_W_Be_a =
      is_binary_ ? c.at<gtsam::Pose3>(keys()[1]) : gtsam::Pose3::Identity();

    const gtsam::Pose3 delta_pose_b_a_Be = T_W_Be_b.inverse() * T_W_Be_a;
    const gtsam::Pose3 delta_pose_b_a_Le =
      config_.T_B_L.inverse() * delta_pose_b_a_Be * config_.T_B_L;

    statuses_.assign(a_features_.size(), RejectStatus::Unprocessed);

    M66 J_b_T_J_b = Z6;
    M66 J_b_T_J_a = Z6;
    M66 J_a_T_J_a = Z6;
    M61 J_b_T_b = Z61;
    M61 J_a_T_b = Z61;
    double f = 0;

    for (size_t feature_id = 0; feature_id < a_features_.size(); feature_id++) {
      // In a photometric factor, each patch now contributes a residual.
      // The patch is a MxM grid of pixels. The residual is NCC diff which is a (MxM)x1 vector

      // Get the feature
      Feature & a_feature = a_features_.at(feature_id);

      // Iterate over the points in the feature
      std::vector<V2D> uv_bs(a_feature.Le_ps.size());
      Eigen::VectorXd I_bs(a_feature.Le_ps.size());
      std::vector<V3D> p_Lk_bs(a_feature.Le_ps.size());
      std::vector<gtsam::Pose3> T_Le_Lts(a_feature.Le_ps.size());

      for (size_t i = 0; i < a_feature.Le_ps.size(); i++) {
        // First we need to find the time at which the LiDAR was looking in the direction of
        // the feature to be able to get T_Le_Lt

        // Transform the point from the a Le frame to the b Le frame
        const V3D p_Le_b = delta_pose_b_a_Le * a_feature.Le_ps[i];
        if (!projectUndistorted(
              p_Le_b, p_Lk_bs[i], uv_bs[i], T_Le_Lts[i], true, b_frame_->proj_idx,
              b_frame_->points_deskewed, b_frame_->interpolated_map_T_Le_Lt, b_frame_->yaw_angles,
              config_)) {
          // The point is not in the image. There is nothing further to do for this feature
          statuses_[feature_id] = RejectStatus::PointProjectUndistorted;
          break;
        }

        // Check if the point is in a valid range
        if (p_Lk_bs[i].norm() < config_.range_min || p_Lk_bs[i].norm() > config_.range_max) {
          statuses_[feature_id] = RejectStatus::PointRange;
          break;
        }

        // Check if the projected point is in the mask
        cv::Point2i uv_b_rounded;
        uv_b_rounded.x = std::round(uv_bs[i].x());
        uv_b_rounded.y = std::round(uv_bs[i].y());
        if (!b_frame_->img_mask.ptr<uchar>(uv_b_rounded.y)[uv_b_rounded.x]) {
          statuses_[feature_id] = RejectStatus::PointMask;
          break;
        }
        if (
          uv_b_rounded.x < config_.margin_size ||
          uv_b_rounded.x >= int(config_.cols) - config_.margin_size ||
          uv_b_rounded.y < config_.margin_size ||
          uv_b_rounded.y >= int(config_.rows) - config_.margin_size) {
          statuses_[feature_id] = RejectStatus::PointMaskMargin;
          break;
        }

        // Check that the range of the point is not too different from the observed range in the range image
        if (
          std::abs(
            b_frame_->img_range.ptr<float>(uv_b_rounded.y)[uv_b_rounded.x] - p_Lk_bs[i].norm()) >
          config_.occlusion_range_diff_threshold) {
          statuses_[feature_id] = RejectStatus::PointRangeDiff;
          break;
        }

        I_bs[i] = getSubPixelValue(b_frame_->img_intensity, uv_bs[i]);
      }

      // Only use the patch if all the points were valid
      if (statuses_[feature_id] != RejectStatus::Unprocessed) {
        continue;
      }

      // Do the NCC
      VXD psi_Ib;
      double mean_I_b, sigma_I_b;
      getPsi(I_bs, mean_I_b, sigma_I_b, psi_Ib);

      // Get the error
      VXD e = psi_Ib - a_feature.psi_intensities;

      const double e_ncc = (2 - e.squaredNorm()) / 2;

      // Check if error is below threshold
      if (e_ncc < config_.max_error) {
        statuses_[feature_id] = RejectStatus::MaxError;
        continue;
      }

      // Set this as a valid feature
      statuses_[feature_id] = RejectStatus::Valid;
      a_feature.center = uv_bs[uv_bs.size() / 2];

      // Compute the Jacobians
      // First compute the part that is separatable

      Eigen::MatrixXd dI_dT_a(a_feature.Le_ps.size(), 6);
      Eigen::MatrixXd dI_dT_b(a_feature.Le_ps.size(), 6);
      for (size_t i = 0; i < a_feature.Le_ps.size(); i++) {
        // Compute the Jacobians
        // Utilize chain rule to separate
        // de/dT = dI/duv * duv/dp * dp/dT

        // First the image gradient
        M12 dI_duv;
        dI_duv << getSubPixelValue(b_frame_->img_dx, uv_bs[i]),
          getSubPixelValue(b_frame_->img_dy, uv_bs[i]);

        // Projection
        M23 duv_dp;
        getProjectionJacobian(p_Lk_bs[i], config_, duv_dp);

        // Transform
        M36 dp_dT_b, dp_dT_a;
        const V3D p_Be_a = config_.T_B_L * a_feature.Le_ps[i];
        const V3D p_Be_b = delta_pose_b_a_Be * p_Be_a;
        const gtsam::Rot3 R_Lk_b_Be_b =
          T_Le_Lts[i].rotation().inverse() * config_.T_B_L.rotation().inverse();

        dp_dT_b.block<3, 3>(0, 0) = R_Lk_b_Be_b.matrix() * gtsam::SO3::Hat(p_Be_b);
        dp_dT_b.block<3, 3>(0, 3) = -R_Lk_b_Be_b.matrix();
        // Compute the final Jacobians
        dI_dT_b.block<1, 6>(i, 0) = dI_duv * duv_dp * dp_dT_b;

        if (is_binary_) {
          const gtsam::Rot3 R_Lk_b_Be_a = R_Lk_b_Be_b * delta_pose_b_a_Be.rotation();
          dp_dT_a.block<3, 3>(0, 0) = -R_Lk_b_Be_a.matrix() * gtsam::SO3::Hat(p_Be_a);
          dp_dT_a.block<3, 3>(0, 3) = R_Lk_b_Be_a.matrix();
          dI_dT_a.block<1, 6>(i, 0) = dI_duv * duv_dp * dp_dT_a;
        }
      }

      MXD J_psi(psi_Ib.size(), psi_Ib.size());
      getPsiJacobian(sigma_I_b, psi_Ib, J_psi);

      // Now multiply it all together to get the final jacobian
      MXD J_final_b = J_psi * dI_dT_b;
      MXD J_final_a;
      if (is_binary_) {
        J_final_a = J_psi * dI_dT_a;
      }

      // Whiten the error
      const double whitened_error = e.norm() / config_.sigma;

      // Get the robust weight
      double sqrt_weight = 1.0;
      if (config_.use_robust_cost_function) {
        if (config_.robust_cost_function == "huber") {
          sqrt_weight = (abs(whitened_error) <= config_.robust_cost_function_parameter)
                          ? 1.0
                          : std::sqrt(config_.robust_cost_function_parameter / abs(whitened_error));
        } else if (config_.robust_cost_function == "gemanmcclure") {
          sqrt_weight =
            config_.robust_cost_function_parameter * config_.robust_cost_function_parameter /
            (config_.robust_cost_function_parameter * config_.robust_cost_function_parameter +
             whitened_error * whitened_error);
        } else {
          throw std::runtime_error("Invalid robust cost function");
        }
      }

      // Whiten and apply robust cost
      J_final_b *= sqrt_weight / config_.sigma;
      e *= sqrt_weight / config_.sigma;

      // Accumulate the components
      J_b_T_J_b += J_final_b.transpose() * J_final_b;
      J_b_T_b += J_final_b.transpose() * e;
      f += e.dot(e);

      if (is_binary_) {
        J_final_a *= sqrt_weight / config_.sigma;
        J_a_T_J_a += J_final_a.transpose() * J_final_a;
        J_b_T_J_a += J_final_b.transpose() * J_final_a;
        J_a_T_b += J_final_a.transpose() * e;
      }
    }

    if (is_binary_) {
      return std::make_shared<gtsam::HessianFactor>(
        keys()[0], keys()[1], J_b_T_J_b, J_b_T_J_a, -J_b_T_b, J_a_T_J_a, -J_a_T_b, f);
    } else {
      auto J_I = J_b_T_J_b;
      auto b_I = J_b_T_b;

      J_b_T_J_b = VSVt * J_I * VSVt;
      J_b_T_b = VSVt * J_I * VSVt * J_I.inverse() * b_I;

      // Get the final localizabilities
      computeLocalizability(
        J_b_T_J_b.topLeftCorner(3, 3), localizability_rot_final_, localizability_eigenvectors_rot_);
      computeLocalizability(
        J_b_T_J_b.bottomRightCorner(3, 3), localizability_trans_final_,
        localizability_eigenvectors_trans_);

      return std::make_shared<gtsam::HessianFactor>(keys()[0], J_b_T_J_b, -J_b_T_b, f);
    }
  }
};
}  // namespace lidar
}  // namespace mimosa
