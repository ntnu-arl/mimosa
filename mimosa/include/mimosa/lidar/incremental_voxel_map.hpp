// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <pcl/point_cloud.h>
#define PCL_NO_PRECOMPILE
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include "mimosa/lidar/point.hpp"

namespace mimosa
{
namespace lidar
{

// This class is a wrapper around the iVox class from gtsam_points to allow using it with PCL point clouds.
class IncrementalVoxelMapPCL
{
public:
  using Ptr = std::shared_ptr<IncrementalVoxelMapPCL>;

private:
  gtsam_points::iVox::Ptr ivox_;

public:
  IncrementalVoxelMapPCL(const float leaf_size);

  // Deep copy constructor
  IncrementalVoxelMapPCL(const IncrementalVoxelMapPCL & other)
  {
    if (other.ivox_) {
      // assumes that gtsam_points::iVox has a proper copy constructor
      ivox_ = std::make_shared<gtsam_points::iVox>(*other.ivox_);
    } else {
      ivox_ = nullptr;
    }
  }

  void insert(const pcl::PointCloud<Point> & cloud);
  bool knn_search(
    const Eigen::Vector3d & point, const size_t k, std::vector<size_t> & indices,
    std::vector<double> & sq_dists, double max_sq_dist = std::numeric_limits<double>::max());
  pcl::PointCloud<Point>::Ptr getCloud();

  gtsam_points::iVox::Ptr underlying() { return ivox_; }
};

gtsam_points::PointCloudCPU::Ptr toGtsamPoints(const pcl::PointCloud<Point> & cloud);
pcl::PointCloud<Point>::Ptr toPcl(const gtsam_points::PointCloudCPU & cloud);

}  // namespace lidar

}  // namespace mimosa
