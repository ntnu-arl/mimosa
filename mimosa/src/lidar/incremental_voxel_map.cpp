// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/lidar/incremental_voxel_map.hpp"

namespace mimosa
{
namespace lidar
{

IncrementalVoxelMapPCL::IncrementalVoxelMapPCL(const float leaf_size)
{
  ivox_ = std::make_shared<gtsam_points::iVox>(leaf_size);
}

void IncrementalVoxelMapPCL::insert(const pcl::PointCloud<Point> & cloud)
{
  // Convert the pointcloud to gtsam_points
  auto frame = toGtsamPoints(cloud);
  ivox_->insert(*frame);
}

bool IncrementalVoxelMapPCL::knn_search(
  const Eigen::Vector3d & point, const size_t k, std::vector<size_t> & indices,
  std::vector<double> & sq_dists, double max_sq_dist)
{
  double pt[3] = {point.x(), point.y(), point.z()};
  return ivox_->knn_search(pt, k, indices.data(), sq_dists.data(), max_sq_dist) == k;
}

pcl::PointCloud<Point>::Ptr IncrementalVoxelMapPCL::getCloud()
{
  auto data = ivox_->voxel_data();
  return toPcl(*data);
}

gtsam_points::PointCloudCPU::Ptr toGtsamPoints(const pcl::PointCloud<Point> & cloud)
{
  std::vector<Eigen::Vector3f> points(cloud.size());
  for (size_t i = 0; i < cloud.size(); i++) {
    points[i] << cloud.points[i].x, cloud.points[i].y, cloud.points[i].z;
  }

  return std::make_shared<gtsam_points::PointCloudCPU>(points);
}

pcl::PointCloud<Point>::Ptr toPcl(const gtsam_points::PointCloudCPU & cloud)
{
  pcl::PointCloud<Point>::Ptr pcl_cloud(new pcl::PointCloud<Point>);
  pcl_cloud->resize(cloud.size());

  for (size_t i = 0; i < cloud.size(); i++) {
    pcl_cloud->points[i].x = cloud.points[i].x();
    pcl_cloud->points[i].y = cloud.points[i].y();
    pcl_cloud->points[i].z = cloud.points[i].z();
  }

  return pcl_cloud;
}

}  // namespace lidar
}  // namespace mimosa
