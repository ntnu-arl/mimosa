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
#include "mimosa/lidar/geometric_config.hpp"
#include "mimosa/lidar/incremental_voxel_map.hpp"
#include "mimosa/state.hpp"

namespace mimosa
{
namespace lidar
{
class ICPFactor : public gtsam::NonlinearFactor
{
  // This is a factor between one or two poses
  // If it is a unary factor, then the key_source is T_W_B and its cloud is registered to the map frame
  // If it is a binary factor, then the key_source is T_W_B_source and its cloud is registered to the cloud from key_target with pose T_W_B_target
  // All clouds are assumed to be deskewed already and in the body frame

public:
  using Ptr = std::shared_ptr<ICPFactor>;

  enum class RejectStatus
  {
    Unprocessed = 0,
    InsufficientCorresPoints,
    CorresMaxDist,
    EigenSolverFail,
    MinEigenValueLow,
    Line,
    CorresPlaneInvalid,
    MaxError,
    Valid
  };

  const std::vector<RejectStatus> & getStatuses() const { return statuses_; }
  const std::vector<V3D> & getCorresMeansTarget() const { return corres_means_target_; }
  const std::vector<V3D> & getCorresNormalsTarget() const { return corres_normals_target_; }

  void getLocalizabilities(
    V3D & trans_comp, V3D & rot_comp, V3D & trans_final, V3D & rot_final, M33 & eigenvectors_trans,
    M33 & eigenvectors_rot)
  {
    trans_comp = localizability_trans_component_;
    rot_comp = localizability_rot_component_;
    trans_final = localizability_trans_final_;
    rot_final = localizability_rot_final_;
    eigenvectors_trans = localizability_eigenvectors_trans_;
    eigenvectors_rot = localizability_eigenvectors_rot_;
  }

  void getDegenInfo(V3D & rot, M33 & eigenvectors_rot, V3D & trans, M33 & eigenvectors_trans)
  {
    rot = degen_info_rot_;
    eigenvectors_rot = degen_eigenvectors_rot_;
    trans = degen_info_trans_;
    eigenvectors_trans = degen_eigenvectors_trans_;
  }

  int getLinearizeCount() const { return linearize_count_; }

private:
  const bool is_binary_;
  IncrementalVoxelMapPCL::Ptr ivox_target_;
  pcl::PointCloud<Point> cloud_source_;

  mutable std::vector<V3D> transed_point_target_;  // Transformed points in target frame
  mutable std::vector<V3D>
    transed_point_target_da_;  // Transformed points in target frame when they were used for data association
  mutable std::vector<V3D> corres_means_target_;    // Mean of the closest points in target frame
  mutable std::vector<V3D> corres_normals_target_;  // Normal of the plane in target frame
  mutable std::vector<RejectStatus> statuses_;      // Statues of each factor

  // QOL Typedefs
  typedef gtsam::NonlinearFactor Base;
  typedef ICPFactor This;
  typedef std::shared_ptr<This> shared_ptr;

  RegistrationConfig config_;

  // Localizabilities
  // Note all of these are whitened and weighted
  mutable V3D
    localizability_trans_component_;  // Localizability of translation computed from the individual components
  mutable V3D
    localizability_rot_component_;  // Localizability of rotation computed from the individual components
  mutable V3D
    localizability_trans_final_;  // Localizability of translation computed from the final JtJ
  mutable V3D localizability_rot_final_;  // Localizability of rotation computed from the final JtJ

  mutable std::vector<V3D>
    localizabilities_trans_body_;  // Localizabilities of translation for each point
  mutable std::vector<V3D>
    localizabilities_rot_body_;  // Localizabilities of rotation for each point

  mutable M33 localizability_eigenvectors_trans_ = M33::Identity();
  mutable M33 localizability_eigenvectors_rot_ = M33::Identity();

  mutable V3D degen_info_rot_;
  mutable V3D degen_info_trans_;
  mutable M33 degen_eigenvectors_rot_;
  mutable M33 degen_eigenvectors_trans_;

  mutable int linearize_count_;

public:
  ICPFactor(
    const gtsam::Key key_source, IncrementalVoxelMapPCL::Ptr ivox_target,
    const pcl::PointCloud<Point> & cloud_source, const RegistrationConfig & config)
  : Base(std::vector<gtsam::Key>{key_source}),
    is_binary_(false),
    ivox_target_(ivox_target),
    cloud_source_(cloud_source),
    config_(config)
  {
    commonConstructor();
  }

  ICPFactor(
    const gtsam::Key key_source, const gtsam::Key key_target,
    IncrementalVoxelMapPCL::Ptr ivox_target, const pcl::PointCloud<Point> & cloud_source,
    const RegistrationConfig & config)
  : Base(std::vector<gtsam::Key>{key_source, key_target}),
    is_binary_(true),
    ivox_target_(ivox_target),
    cloud_source_(cloud_source),
    config_(config)
  {
    commonConstructor();
  }

  void commonConstructor()
  {
    // These need to be the size of the source since we are iterating over the source
    transed_point_target_.resize(cloud_source_.size(), Z31);
    transed_point_target_da_.resize(cloud_source_.size(), Z31);
    corres_means_target_.resize(cloud_source_.size(), Z31);
    corres_normals_target_.resize(cloud_source_.size(), Z31);
    statuses_.resize(cloud_source_.size(), RejectStatus::Unprocessed);
    localizabilities_trans_body_.resize(cloud_source_.size(), Z31);
    localizabilities_rot_body_.resize(cloud_source_.size(), Z31);

    linearize_count_ = 0;
  }

  ~ICPFactor() override {}

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
              << gdkf(keys()[0])
              << " with value: " << c.at<gtsam::Pose3>(keys()[0]) << "\n";
    return 0.0;
  }

  bool estimatePlane(
    const size_t i, const std::vector<size_t> & closest_point_indices,
    const V3D & source_origin_in_target) const
  {
    // Fill in the points
    MX3D points(config_.num_corres_points, 3);
    for (size_t j = 0; j < config_.num_corres_points; ++j) {
      points.block<1, 3>(j, 0) =
        ivox_target_->underlying()->point(closest_point_indices[j]).head<3>();
    }

    // Estimate the plane by solving the system of n.p/(n.j) = 1
    MX1D ones(config_.num_corres_points, 1);
    ones.setOnes();

    corres_means_target_[i] = points.colwise().mean();

    const MX3D centered = points.rowwise() - corres_means_target_[i].transpose();
    const M3D cov = centered.transpose() * centered / (centered.rows() - 1);

    Eigen::SelfAdjointEigenSolver<M3D> eigensolver(cov);
    if (eigensolver.info() != Eigen::Success) {
      statuses_[i] = RejectStatus::EigenSolverFail;
      return false;
    }

    if (eigensolver.eigenvalues().x() < 1e-6) {
      // Unrealistic Plane
      statuses_[i] = RejectStatus::MinEigenValueLow;
      return false;
    }

    // Check for accidental line
    if (eigensolver.eigenvalues().z() > 3 * eigensolver.eigenvalues().y()) {
      // This is a line
      statuses_[i] = RejectStatus::Line;
      return false;
    }

    corres_normals_target_[i] = eigensolver.eigenvectors().col(0);

    // The normal should always point towards the robot - this is for visualization
    if (corres_normals_target_[i].dot(source_origin_in_target - corres_means_target_[i]) < 0) {
      corres_normals_target_[i] *= -1;
    }

    if (((centered * corres_normals_target_[i]).array().abs() > config_.plane_validity_distance)
          .any()) {
      statuses_[i] = RejectStatus::CorresPlaneInvalid;
      return false;
    }

    return true;
  }

  std::shared_ptr<gtsam::GaussianFactor> linearize(const gtsam::Values & c) const override
  {
    linearize_count_++;

    // std::cout << "calling linearize key: " << gdkf(keys()[0]) << std::endl;
    // Caching with call_count is wrong because relinearization happens before new factors are added
    // Therefore, if we did cache a gaussian factor it would be at an old linearization point
    // Instead, it is possible to do the correspondence calculation before, since it is unlikely that
    // this changes significantly between linearizations. Additionally, the correspondence calculation
    // is the most expensive part of the linearization due to the kd-tree search.
    // Additionally, since we need localizability before optimization, the linearize function
    // should be called right after creating the factor.

    // Most basically, iterate over the points in the cloud_source, transform them to the target frame
    // find the correspondence, and compute the error

    const gtsam::Pose3 & T_W_B_source = c.at<gtsam::Pose3>(keys()[0]);
    const gtsam::Pose3 T_W_B_target =
      is_binary_ ? c.at<gtsam::Pose3>(keys()[1]) : gtsam::Pose3::Identity();

    const gtsam::Pose3 delta_pose = T_W_B_target.inverse() * T_W_B_source;

    const V3D source_origin_in_target = delta_pose * V3D::Zero();

    // The 4 DoF registration is a special case where the factor does not constrain global roll and pitch
    // const V3D global_z(0, 0, 1);
    const V3D global_z = -(c.at<gtsam::Unit3>(G(0))).unitVector();
    const V3D local_z = delta_pose.rotation().inverse() * global_z;
    const M33 rot_projection_mat = local_z * local_z.transpose();

    size_t n_threads = 4;

    std::vector<M66> J_source_T_J_source_thread(n_threads, Z6);
    std::vector<M66> J_source_T_J_target_thread(n_threads, Z6);
    std::vector<M66> J_target_T_J_target_thread(n_threads, Z6);
    std::vector<M61> J_source_T_b_thread(n_threads, Z61);
    std::vector<M61> J_target_T_b_thread(n_threads, Z61);
    std::vector<double> f_thread(n_threads, 0);

    std::vector<M16> J_source_whitened_weighted_arr(cloud_source_.size(), Z16);
    std::vector<double> e_whitened_weigehted_arr(cloud_source_.size(), 0);
#if USE_OPENMP
#pragma omp parallel for num_threads(n_threads)
#endif
    for (size_t i = 0; i < cloud_source_.points.size(); i++) {
      transed_point_target_[i] =
        delta_pose * cloud_source_.points[i].getVector3fMap().cast<double>();

      bool update_correspondance = false;
      // While a denominator of 2 * config_.num_corres_points provides provable guarantees, this heuristic also works well
      if (
        (transed_point_target_[i] - transed_point_target_da_[i]).norm() >
        config_.target_ivox_map_min_dist_in_voxel / 4) {
        // The point is far from the previous linearize location so the correspondences have to be updated
        update_correspondance = true;
        transed_point_target_da_[i] = transed_point_target_[i];
      }

      if (update_correspondance) {
        statuses_[i] = RejectStatus::Unprocessed;

        std::vector<size_t> k_indices(config_.num_corres_points);
        std::vector<double> sq_dists(config_.num_corres_points);
        if (!ivox_target_->knn_search(
              transed_point_target_[i], config_.num_corres_points, k_indices, sq_dists)) {
          statuses_[i] = RejectStatus::InsufficientCorresPoints;
          continue;
        }
        if (sq_dists.back() > config_.max_corres_distance * config_.max_corres_distance) {
          statuses_[i] = RejectStatus::CorresMaxDist;
          continue;
        }

        // Estimate the correspondence plane
        if (!estimatePlane(i, k_indices, source_origin_in_target)) {
          continue;
        }
      } else {
        // The point has not moved enough to redo data association
        // However, the correspondence that happened in the previous iteration might
        // have been rejected. Additionally, the point may also have been max error rejected.
        // Therefore only points that were valid or were rejected due to checks after this line
        // in the previous iteration should be considered
        if (statuses_[i] <= RejectStatus::CorresPlaneInvalid) {
          continue;
        }
      }

      double e = corres_normals_target_[i].dot(corres_means_target_[i] - transed_point_target_[i]);

      // s check
      double s =
        1 - 0.9 * abs(e) / sqrt(cloud_source_.points[i].getVector3fMap().cast<double>().norm());

      if (s < 0.9) {
        statuses_[i] = RejectStatus::MaxError;
        continue;
      }

      // Compute weight
      double sqrt_weight = 1.0;
      if (config_.use_huber) {
        double whitened_error = e / config_.lidar_point_noise_std_dev;
        if (abs(whitened_error) > config_.huber_threshold) {
          sqrt_weight = std::sqrt(config_.huber_threshold / abs(whitened_error));
        }
      }

      e *= sqrt_weight / config_.lidar_point_noise_std_dev;

      // Compute the Jacobians
      // Source
      V3D corres_normals_source = delta_pose.rotation().inverse() * corres_normals_target_[i];
      M16 J_source;
      J_source.block<1, 3>(0, 0) =
        (corres_normals_source.cross(cloud_source_.points[i].getVector3fMap().cast<double>()))
          .transpose();
      J_source.block<1, 3>(0, 3) = -corres_normals_source.transpose();

      // Get the localizabilties
      localizabilities_rot_body_[i] = J_source.block<1, 3>(0, 0).normalized();
      localizabilities_trans_body_[i] = J_source.block<1, 3>(0, 3);

      // Whiten and weight
      J_source *= sqrt_weight / config_.lidar_point_noise_std_dev;

#if USE_OPENMP
      const int thread_id = omp_get_thread_num();
#else
      const int thread_id = 0;
#endif

      // Accumulate the Hessian
      J_source_T_J_source_thread[thread_id] += J_source.transpose() * J_source;
      J_source_T_b_thread[thread_id] += J_source.transpose() * e;
      f_thread[thread_id] += e * e;

      if (is_binary_) {
        // Target
        M16 J_target;
        J_target.block<1, 3>(0, 0) =
          (transed_point_target_[i].cross(corres_normals_target_[i])).transpose();
        J_target.block<1, 3>(0, 3) = corres_normals_target_[i].transpose();

        // Whiten and weight
        J_target *= sqrt_weight / config_.lidar_point_noise_std_dev;

        // Accumulate the Hessian
        J_source_T_J_target_thread[thread_id] += J_source.transpose() * J_target;
        J_target_T_J_target_thread[thread_id] += J_target.transpose() * J_target;
        J_target_T_b_thread[thread_id] += J_target.transpose() * e;
      }

      // Set the status to valid
      statuses_[i] = RejectStatus::Valid;
    }

    // Combine the thread results
    M66 J_source_T_J_source = Z6;
    M66 J_source_T_J_target = Z6;
    M66 J_target_T_J_target = Z6;
    M61 J_source_T_b = Z61;
    M61 J_target_T_b = Z61;
    double f = 0;

    for (size_t i = 0; i < n_threads; i++) {
      J_source_T_J_source += J_source_T_J_source_thread[i];
      J_source_T_J_target += J_source_T_J_target_thread[i];
      J_target_T_J_target += J_target_T_J_target_thread[i];
      J_source_T_b += J_source_T_b_thread[i];
      J_target_T_b += J_target_T_b_thread[i];
      f += f_thread[i];
    }

    // Get the final localizabilities
    computeLocalizability(
      J_source_T_J_source.topLeftCorner(3, 3), localizability_rot_final_,
      localizability_eigenvectors_rot_);
    computeLocalizability(
      J_source_T_J_source.bottomRightCorner(3, 3), localizability_trans_final_,
      localizability_eigenvectors_trans_);

    M33 Sigma_rr = (J_source_T_J_source.topLeftCorner(3, 3) -
                    J_source_T_J_source.topRightCorner(3, 3) *
                      J_source_T_J_source.bottomRightCorner(3, 3).inverse() *
                      J_source_T_J_source.bottomLeftCorner(3, 3))
                     .inverse();
    M33 Sigma_tt = (J_source_T_J_source.bottomRightCorner(3, 3) -
                    J_source_T_J_source.bottomLeftCorner(3, 3) *
                      J_source_T_J_source.topLeftCorner(3, 3).inverse() *
                      J_source_T_J_source.topRightCorner(3, 3))
                     .inverse();

    // Get the degen infos
    computeLocalizability(Sigma_rr, degen_info_rot_, degen_eigenvectors_rot_);
    computeLocalizability(Sigma_tt, degen_info_trans_, degen_eigenvectors_trans_);
    // Convert rotational degen info to degrees
    degen_info_rot_ = RAD2DEG(degen_info_rot_);

    // The localizabilities through components are computed in the body frame, however, the final localizabilities
    // are in a basis formed by the eigenvectors. Therefore, the components need to be transformed to this basis.
    // This can be done by simply taking the dot product of individual localizabilities with the eigenvectors and
    // and adding the absolute values
    localizability_trans_component_ = Z31;
    localizability_rot_component_ = Z31;
    for (size_t i = 0; i < localizabilities_trans_body_.size(); ++i) {
      if (statuses_[i] != RejectStatus::Valid) {
        continue;
      }

      V3D trans_comp =
        (localizabilities_trans_body_[i].transpose() * localizability_eigenvectors_trans_)
          .transpose()
          .cwiseAbs();
      // Zero out the values of trans_comp that are less than 0.5
      trans_comp = (trans_comp.array() >= 0.5).select(trans_comp, 0);

      localizability_trans_component_ += trans_comp;

      V3D rot_comp = (localizabilities_rot_body_[i].transpose() * localizability_eigenvectors_rot_)
                       .transpose()
                       .cwiseAbs();
      // Zero out the values of rot_comp that are less than 0.5
      rot_comp = (rot_comp.array() >= 0.5).select(rot_comp, 0);

      localizability_rot_component_ += rot_comp;
    }

    if (is_binary_) {
      return std::make_shared<gtsam::HessianFactor>(
        keys()[0], keys()[1], J_source_T_J_source, J_source_T_J_target, -J_source_T_b,
        J_target_T_J_target, -J_target_T_b, f);
    } else {
      if (config_.reg_4_dof)
      {
        // Project the matrices
        J_source_T_J_source.block<3,3>(0,0) = rot_projection_mat * J_source_T_J_source.block<3,3>(0,0) * rot_projection_mat;
        J_source_T_J_source.block<3,3>(0,3) = rot_projection_mat * J_source_T_J_source.block<3,3>(0,3);
        J_source_T_J_source.block<3,3>(3,0) = J_source_T_J_source.block<3,3>(3,0) * rot_projection_mat;

        // Project J_T_b
        J_source_T_b.head<3>() = rot_projection_mat * J_source_T_b.head<3>();
      }

      if (config_.project_on_degneneracy) {
        M33 P_rot;
        V3D degenerate_axes_rot;
        bool rot_degen = getProjectionMatrix(
          localizability_rot_final_, config_.degen_thresh_rot, localizability_eigenvectors_rot_,
          P_rot, degenerate_axes_rot);

        M33 P_trans;
        V3D degenerate_axes_trans;
        bool trans_degen = getProjectionMatrix(
          localizability_trans_final_, config_.degen_thresh_trans,
          localizability_eigenvectors_trans_, P_trans, degenerate_axes_trans);

        // If either of these are degenerate, then we need to remap the J_T_J and J_T_b
        if (rot_degen || trans_degen) {
          std::cout << "*********************DEGEN Handling*********************" << std::endl;
          std::cout << "P_rot:\n" << P_rot << std::endl;
          std::cout << "P_trans:\n" << P_trans << std::endl;

          J_source_T_J_source = Z66;
          J_source_T_b = Z61;

          // The weak features should be projected
          for (size_t i = 0; i < cloud_source_.size(); i++) {
            if (statuses_[i] != RejectStatus::Valid) {
              continue;
            }

            // Project the localizabilities
            // J is simply [(p_B x n_B)^T, n_B^T]

            // Get the projection of J in the eigenvector basis to see whether it has a strong or weak contribution
            // If this is weak (i.e. within possible SNR), then it should be projected
            // If it is strong, then it should not be projected
            bool weak = false;

            const double weak_threhsold = std::cos(DEG2RAD(70));
            // Translation
            V3D contributions = (J_source_whitened_weighted_arr[i].block<1, 3>(0, 3) *
                                 localizability_eigenvectors_trans_)
                                  .transpose();

            if (std::abs(contributions[0]) < weak_threhsold) {
              weak = true;
            }

            if (weak) {
              J_source_whitened_weighted_arr[i].block<1, 3>(0, 3) =
                J_source_whitened_weighted_arr[i].block<1, 3>(0, 3) * P_trans;
            }

            J_source_T_J_source +=
              J_source_whitened_weighted_arr[i].transpose() * J_source_whitened_weighted_arr[i];
            J_source_T_b +=
              J_source_whitened_weighted_arr[i].transpose() * e_whitened_weigehted_arr[i];
          }

          // Note since P is symmetric, the code drops the transpose
          // // Project J_T_J
          // J_source_T_J_source.block<3, 3>(0, 0) =
          //   P_rot * J_source_T_J_source.block<3, 3>(0, 0) * P_rot;
          // J_source_T_J_source.block<3, 3>(0, 3) =
          //   P_rot * J_source_T_J_source.block<3, 3>(0, 3) * P_trans;
          // J_source_T_J_source.block<3, 3>(3, 0) =
          //   P_trans * J_source_T_J_source.block<3, 3>(3, 0) * P_rot;
          // J_source_T_J_source.block<3, 3>(3, 3) =
          //   P_trans * J_source_T_J_source.block<3, 3>(3, 3) * P_trans;

          // // Project J_T_b
          // J_source_T_b.head<3>() = P_rot * J_source_T_b.head<3>();
          // J_source_T_b.tail<3>() = P_trans * J_source_T_b.tail<3>();

          // Recompute the localizabilities
          computeLocalizability(
            J_source_T_J_source.topLeftCorner(3, 3), localizability_rot_final_,
            localizability_eigenvectors_rot_);
          computeLocalizability(
            J_source_T_J_source.bottomRightCorner(3, 3), localizability_trans_final_,
            localizability_eigenvectors_trans_);
        }
      }

      return std::make_shared<gtsam::HessianFactor>(
        keys()[0], J_source_T_J_source, -J_source_T_b, f);
    }
  }
};

}  // namespace lidar
}  // namespace mimosa
