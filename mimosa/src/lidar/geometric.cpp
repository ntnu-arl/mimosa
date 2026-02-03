// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#include "mimosa/lidar/geometric.hpp"

namespace mimosa
{
namespace lidar
{
Geometric::Geometric(ros::NodeHandle & pnh)
: config(config::checkValid(config::fromRos<GeometricConfig>(pnh)))
{
  // Prepare config
  logger_ = createLogger(
    config.logs_directory + "lidar_geometric.log", "lidar::Geometric", config.log_level);
  logger_->info("lidar::Geometric initialized with params:\n {}", config::toString(config));

  Be_cloud_.reset(new pcl::PointCloud<Point>);

  ivox_map_ =
    std::make_shared<IncrementalVoxelMapPCL>(config.scan_to_map.target_ivox_map_leaf_size);
  ivox_map_->underlying()->set_lru_horizon(config.lru_horizon);
  ivox_map_->underlying()->set_neighbor_voxel_mode(config.neighbor_voxel_mode);
  ivox_map_->underlying()->voxel_insertion_setting().set_min_dist_in_cell(
    config.scan_to_map.target_ivox_map_min_dist_in_voxel);

  pub_sm_cloud_ = pnh.advertise<sensor_msgs::PointCloud2>("lidar/geometric/sm_cloud", 1);
  pub_sm_cloud_ds_ = pnh.advertise<sensor_msgs::PointCloud2>("lidar/geometric/sm_cloud_ds", 1);
  pub_sm_correspondances_ma_ =
    pnh.advertise<visualization_msgs::MarkerArray>("lidar/geometric/sm_correspondances_ma", 1);
  pub_debug_ = pnh.advertise<mimosa_msgs::LidarGeometricDebug>("lidar/geometric/debug", 1);
  pub_map_ = pnh.advertise<sensor_msgs::PointCloud2>("lidar/geometric/map", 1);
  pub_localizability_marker_array_ = pnh.advertise<visualization_msgs::MarkerArray>(
    "lidar/geometric/localizability_marker_array", 1, true);
  pub_degen_marker_array_ =
    pnh.advertise<visualization_msgs::MarkerArray>("lidar/geometric/degen_marker_array", 1, true);
  pub_keyframe_poses_ =
    pnh.advertise<geometry_msgs::PoseArray>("lidar/geometric/keyframe_poses", 1);

  keyframe_poses_.header.frame_id = config.world_frame;
}

/**
 * @brief Performs one-shot voxel grid downsampling using the iVox logic.
 * @tparam PointT The PCL point type (e.g., pcl::PointXYZ, pcl::PointXYZI).
 * @param input_cloud The input point cloud.
 * @param leaf_size The size of the voxel cube side.
 * @param max_points_per_voxel Maximum number of points to keep within a single voxel.
 * @param min_dist_in_voxel Minimum distance between points kept within the same voxel.
 * @return A new point cloud containing the downsampled points.
 */
void Geometric::downsample(
  const pcl::PointCloud<Point> & input_cloud, pcl::PointCloud<Point> & output_cloud,
  const double leaf_size, const size_t max_points_per_voxel, const double min_dist_in_voxel)
{
  Stopwatch sw;

  const double inv_leaf_size = 1.0 / leaf_size;
  FlatContainerMinimal::Setting voxel_settings;
  voxel_settings.max_num_points_in_cell = max_points_per_voxel;
  voxel_settings.min_sq_dist_in_cell = min_dist_in_voxel * min_dist_in_voxel;

  // Map from voxel coordinates to the container holding points for that voxel
  std::vector<FlatContainerMinimal> flat_voxels;
  flat_voxels.reserve(input_cloud.size() / 2);
  std::unordered_map<Eigen::Vector3i, size_t, XORVector3iHash> voxels;
  voxels.reserve(input_cloud.size() / 2);

  debug_msg_.t_preprocess1 = sw.tickMs();

  // Iterate through input points and assign them to voxels
  for (size_t i = 0; i < input_cloud.size(); ++i) {
    // Get point coordinates as Eigen vector
    // Assuming PointT has x, y, z members
    const Eigen::Vector4d point_pos = input_cloud.points[i].getVector4fMap().cast<double>();

    // Calculate integer voxel coordinates
    const Eigen::Vector3i coord = fast_floor(point_pos * inv_leaf_size).head<3>();

    auto found = voxels.find(coord);
    if (found == voxels.end()) {
      // If the voxel doesn't exist, create a new one
      found = voxels.emplace_hint(found, coord, flat_voxels.size());
      flat_voxels.emplace_back(voxel_settings);
    }

    flat_voxels[found->second].add(voxel_settings, point_pos.head<3>(), i);
  }

  debug_msg_.t_preprocess2 = sw.tickMs();

  // Collect points from all voxels
  // Reserve space roughly - this is an estimate
  std::vector<size_t> indices;
  indices.reserve(voxels.size() * max_points_per_voxel);
  for (const auto & voxel : flat_voxels) {
    indices.insert(indices.end(), voxel.get_indices().begin(), voxel.get_indices().end());
  }

  // Resize the output cloud to fit the selected points
  output_cloud.clear();
  output_cloud.reserve(indices.size());
  for (const size_t idx : indices) {
    output_cloud.points.emplace_back(input_cloud.points[idx]);
  }
  // Set the output cloud properties
  output_cloud.header = input_cloud.header;
  output_cloud.is_dense = true;
  output_cloud.width = output_cloud.size();
  output_cloud.height = 1;

  debug_msg_.t_preprocess3 = sw.tickMs();
}

void Geometric::preprocess(
  const pcl::PointCloud<Point> & points_deskewed, const std::vector<size_t> & idxs, const double ts)
{
  if (!config.enabled) return;

  Stopwatch sw;
  logger_->trace("Preprocess start");

  ts_ = ts;

  // Clear the previous cloud
  Be_cloud_->clear();
  Be_cloud_->reserve(idxs.size());

  const M3F R_B_L = config.T_B_L.rotation().matrix().cast<float>();
  const V3F t_B_L = config.T_B_L.translation().cast<float>();

  std::for_each(idxs.begin(), idxs.end(), [&](const size_t idx) {
    // Transform the point from the Le frame to the Be frame
    auto & p = Be_cloud_->points.emplace_back(points_deskewed[idx]);
    p.getVector3fMap() = R_B_L * p.getVector3fMap() + t_B_L;
  });

  if (pub_sm_cloud_.getNumSubscribers()) {
    publishCloud(pub_sm_cloud_, *Be_cloud_, config.body_frame, ts_);
  }

  // Set the sizes since these are invalidated by the above loop that works directly on the points
  Be_cloud_->width = Be_cloud_->size();
  Be_cloud_->height = 1;

  downsample(
    *Be_cloud_, sm_Be_cloud_ds_, config.scan_to_map.source_voxel_grid_filter_leaf_size, 20,
    config.scan_to_map.source_voxel_grid_min_dist_in_voxel);

  if (pub_sm_cloud_ds_.getNumSubscribers()) {
    publishCloud(pub_sm_cloud_ds_, sm_Be_cloud_ds_, config.body_frame, ts_);
  }

  debug_msg_.n_points_in = points_deskewed.size();
  debug_msg_.n_points_in_sm_ds = sm_Be_cloud_ds_.size();

  logger_->trace("Preprocess end");
  debug_msg_.t_preprocess = sw.elapsedMs();
}

void Geometric::getFactors(
  const gtsam::Key & key, const gtsam::Values & values, gtsam::NonlinearFactorGraph & graph,
  M66 & eigenvectors_block_matrix, V6D & degen_directions)
{
  if (!config.enabled) return;

  Stopwatch sw;

  // Scan to map
  factor_ = std::make_shared<ICPFactor>(key, ivox_map_, sm_Be_cloud_ds_, config.scan_to_map);
  // The factor_ is linearized so that localizability can be computed
  auto tmp = factor_->linearize(values);

  if (pub_sm_correspondances_ma_.getNumSubscribers()) {
    visualization_msgs::MarkerArray ma;
    fillMarkerArray(*factor_, ma, config.world_frame, ts_);
    pub_sm_correspondances_ma_.publish(ma);
  }

  // Get the localizability
  V3D localizability_trans_comp, localizability_rot_comp, localizability_trans_final,
    localizability_rot_final;
  M33 eigenvectors_trans, eigenvectors_rot;
  factor_->getLocalizabilities(
    localizability_trans_comp, localizability_rot_comp, localizability_trans_final,
    localizability_rot_final, eigenvectors_trans, eigenvectors_rot);

  {
    // M33 degen_eigenvectors_trans, degen_eigenvectors_rot;
    // V3D degen_rot, degen_trans;

    // factor_->getDegenInfo(degen_rot, degen_eigenvectors_rot, degen_trans, degen_eigenvectors_trans);

    eigenvectors_block_matrix.setZero();
    eigenvectors_block_matrix.block<3, 3>(0, 0) = eigenvectors_rot;
    eigenvectors_block_matrix.block<3, 3>(3, 3) = eigenvectors_trans;

    degen_directions.setZero();
    degen_directions(0) = localizability_rot_comp(0) < config.scan_to_map.degen_thresh_rot;
    degen_directions(1) = localizability_rot_comp(1) < config.scan_to_map.degen_thresh_rot;
    degen_directions(2) = localizability_rot_comp(2) < config.scan_to_map.degen_thresh_rot;
    degen_directions(3) = localizability_trans_comp(0) < config.scan_to_map.degen_thresh_trans;
    degen_directions(4) = localizability_trans_comp(1) < config.scan_to_map.degen_thresh_trans;
    degen_directions(5) = localizability_trans_comp(2) < config.scan_to_map.degen_thresh_trans;

    // debug_msg_.degen_rot_val[0] = degen_rot(0);
    // debug_msg_.degen_rot_val[1] = degen_rot(1);
    // debug_msg_.degen_rot_val[2] = degen_rot(2);
    // debug_msg_.degen_trans_val[0] = degen_trans(0);
    // debug_msg_.degen_trans_val[1] = degen_trans(1);
    // debug_msg_.degen_trans_val[2] = degen_trans(2);
    debug_msg_.degen_rot_bool[0] = degen_directions(0);
    debug_msg_.degen_rot_bool[1] = degen_directions(1);
    debug_msg_.degen_rot_bool[2] = degen_directions(2);
    debug_msg_.degen_trans_bool[0] = degen_directions(3);
    debug_msg_.degen_trans_bool[1] = degen_directions(4);
    debug_msg_.degen_trans_bool[2] = degen_directions(5);

    // logger_->info("degen_directions: {}", degen_directions.transpose());

    if (pub_degen_marker_array_.getNumSubscribers()) {
      // Create the marker for the axis
      visualization_msgs::MarkerArray ma;
      addTriadMarker(eigenvectors_trans, config.body_frame, ts_, "DegenTrans", ma);
      addTriadMarker(eigenvectors_rot, config.body_frame, ts_, "DegenRot", ma);

      pub_degen_marker_array_.publish(ma);
    }
  }

  // // Enforce convention of positive x for the leading eigenvector
  // // This does not make a difference to the localizability
  // if (eigenvectors_trans(0, 0) < 0) {
  //   // Flip all the eigenvectors
  //   eigenvectors_trans *= -1;
  // }
  // if (eigenvectors_rot(0, 0) < 0) {
  //   // Flip all the eigenvectors
  //   eigenvectors_rot *= -1;
  // }

  if (pub_localizability_marker_array_.getNumSubscribers()) {
    // Create the marker for the axis
    visualization_msgs::MarkerArray ma;
    addTriadMarker(eigenvectors_trans, config.body_frame, ts_, "LocalizabilityTrans", ma);
    addTriadMarker(eigenvectors_rot, config.body_frame, ts_, "LocalizabilityRot", ma);

    pub_localizability_marker_array_.publish(ma);
  }

  convert(localizability_trans_comp, debug_msg_.localizability_trans_comp);
  convert(localizability_rot_comp, debug_msg_.localizability_rot_comp);
  convert(localizability_trans_final, debug_msg_.localizability_trans_final);
  convert(localizability_rot_final, debug_msg_.localizability_rot_final);

  const std::vector<ICPFactor::RejectStatus> & statuses = factor_->getStatuses();
  debug_msg_.n_unprocessed = 0;
  debug_msg_.n_rejected_insufficient_corres_points = 0;
  debug_msg_.n_rejected_max_corres_dist = 0;
  debug_msg_.n_rejected_eigensolver_fail = 0;
  debug_msg_.n_rejected_min_eigen_value_low = 0;
  debug_msg_.n_rejected_line = 0;
  debug_msg_.n_rejected_plane = 0;
  debug_msg_.n_rejected_max_error = 0;
  debug_msg_.n_correspondances = 0;
  for (size_t i = 0; i < statuses.size(); i++) {
    switch (statuses[i]) {
      case ICPFactor::RejectStatus::Unprocessed:
        debug_msg_.n_unprocessed++;
        break;
      case ICPFactor::RejectStatus::InsufficientCorresPoints:
        debug_msg_.n_rejected_insufficient_corres_points++;
        break;
      case ICPFactor::RejectStatus::CorresMaxDist:
        debug_msg_.n_rejected_max_corres_dist++;
        break;
      case ICPFactor::RejectStatus::EigenSolverFail:
        debug_msg_.n_rejected_eigensolver_fail++;
        break;
      case ICPFactor::RejectStatus::MinEigenValueLow:
        debug_msg_.n_rejected_min_eigen_value_low++;
        break;
      case ICPFactor::RejectStatus::Line:
        debug_msg_.n_rejected_line++;
        break;
      case ICPFactor::RejectStatus::CorresPlaneInvalid:
        debug_msg_.n_rejected_plane++;
        break;
      case ICPFactor::RejectStatus::MaxError:
        debug_msg_.n_rejected_max_error++;
        break;
      case ICPFactor::RejectStatus::Valid:
        debug_msg_.n_correspondances++;
        break;

      default:
        break;
    }
  }

  graph.add(factor_);

  debug_msg_.t_get_factors = sw.elapsedMs();
}

void Geometric::fillMarkerArray(
  const ICPFactor & factor, visualization_msgs::MarkerArray & ma, const std::string & frame_id,
  const double ts)
{
  const std::vector<ICPFactor::RejectStatus> & statuses = factor.getStatuses();
  const std::vector<V3D> & corres_means_target = factor.getCorresMeansTarget();
  const std::vector<V3D> & corres_normals_target = factor.getCorresNormalsTarget();

  visualization_msgs::Marker triangles;
  triangles.header.frame_id = frame_id;
  triangles.header.stamp.fromSec(ts);
  triangles.ns = "planes";
  triangles.id = 0;
  triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;
  triangles.action = visualization_msgs::Marker::ADD;
  triangles.pose.orientation.w = 1.0;
  triangles.scale.x = 1.0;
  triangles.scale.y = 1.0;
  triangles.scale.z = 1.0;
  triangles.color.r = 0.0;
  triangles.color.g = 1.0;
  triangles.color.b = 0.0;
  triangles.color.a = 1.0;

  visualization_msgs::Marker normals;
  normals.header.frame_id = frame_id;
  normals.header.stamp.fromSec(ts);
  normals.ns = "normals";
  normals.id = 0;
  normals.type = visualization_msgs::Marker::LINE_LIST;
  normals.action = visualization_msgs::Marker::ADD;
  normals.pose.orientation.w = 1.0;
  normals.scale.x = 0.01;
  normals.color.r = 1.0;
  normals.color.g = 0.0;
  normals.color.b = 0.0;
  normals.color.a = 1.0;

  float triangle_size = config.scan_to_map.source_voxel_grid_filter_leaf_size;
  float normal_length = config.scan_to_map.source_voxel_grid_filter_leaf_size;

  for (size_t i = 0; i < corres_means_target.size(); i++) {
    if (statuses[i] != ICPFactor::RejectStatus::Valid) {
      continue;
    }

    // Current point and normal
    const Eigen::Vector3d & p = corres_means_target[i];
    const Eigen::Vector3d & n = corres_normals_target[i];

    // Create vertices for the triangle with centroid p
    Eigen::Vector3d u = n.unitOrthogonal();       // Vector orthogonal to the normal
    Eigen::Vector3d v = n.cross(u).normalized();  // Another orthogonal vector

    // Scale the vectors to half the triangle size (since p is the centroid)
    u *= triangle_size / 2.0;
    v *= triangle_size / 2.0;

    // Vertices of the triangle
    Eigen::Vector3d p1 = p + u + v;
    Eigen::Vector3d p2 = p - u + v;
    Eigen::Vector3d p3 = p - u - v;

    // Add the three vertices of the triangle
    geometry_msgs::Point vertex;
    vertex.x = p1.x();
    vertex.y = p1.y();
    vertex.z = p1.z();
    triangles.points.push_back(vertex);

    vertex.x = p2.x();
    vertex.y = p2.y();
    vertex.z = p2.z();
    triangles.points.push_back(vertex);

    vertex.x = p3.x();
    vertex.y = p3.y();
    vertex.z = p3.z();
    triangles.points.push_back(vertex);

    // Add the normal line (from point to point + normal)
    geometry_msgs::Point start, end;
    start.x = p.x();
    start.y = p.y();
    start.z = p.z();
    end.x = p.x() + n.x() * normal_length;
    end.y = p.y() + n.y() * normal_length;
    end.z = p.z() + n.z() * normal_length;

    normals.points.push_back(start);
    normals.points.push_back(end);
  }

  ma.markers.push_back(triangles);
  ma.markers.push_back(normals);
}

void Geometric::updateMap(const gtsam::Key key, const gtsam::Values & values)
{
  if (!config.enabled) return;

  Stopwatch sw;

  if (factor_ != nullptr) {
    debug_msg_.n_linearize_calls = factor_->getLinearizeCount();
  }

  const gtsam::Pose3 & T_W_Be = values.at<gtsam::Pose3>(key);

  bool update_map = true;
  if (map_poses_.size()) {
    // Check if the pose has changed significantly with respect to the poses that are already in the map
    float min_diff_trans = std::numeric_limits<float>::max();
    size_t min_diff_index = 0;
    size_t pose_index = 0;
    for (const auto & pose : map_poses_) {
      float diff_trans = (pose.translation() - T_W_Be.translation()).norm();
      if (diff_trans < min_diff_trans) {
        min_diff_trans = diff_trans;
        min_diff_index = pose_index;
      }
      pose_index++;
    }

    auto rot_diff = config.T_B_L.rotation().inverse() *
                    map_poses_[min_diff_index].rotation().between(T_W_Be.rotation()) *
                    config.T_B_L.rotation();

    V3D ypr = rot_diff.ypr().cwiseAbs();

    // Rules for global map update:
    // 1. First update should always happen
    // 2. If there is significant difference in the information content of the scan with respect to what is already in the map
    //        - Determining this is hard, but we use the change in the pose as a proxy
    // If the pose is close to an existing pose, then do not update the map
    if (min_diff_trans > config.map_keyframe_trans_thresh) {
      update_map = true;
    } else if (ypr.maxCoeff() > DEG2RAD(config.map_keyframe_rot_thresh_deg)) {
      update_map = true;
    } else {
      update_map = false;
    }
  }

  static size_t initial_clouds_to_force_map_update = config.initial_clouds_to_force_map_update;
  if (initial_clouds_to_force_map_update > 0) {
    update_map = true;
    initial_clouds_to_force_map_update--;
  }

  if (update_map) {
    // Transform the cloud to world frame using the T_W_Be
    Stopwatch sw;
    const M3F R_W_Be = T_W_Be.rotation().matrix().cast<float>();
    const V3F t_W_Be = T_W_Be.translation().cast<float>();

    // Transform and insert the cloud into the map
    std::vector<V3F> W_points(Be_cloud_->size());
    for (size_t i = 0; i < Be_cloud_->size(); i++) {
      W_points[i] = R_W_Be * Be_cloud_->points[i].getVector3fMap() + t_W_Be;
    }
    gtsam_points::PointCloudCPU::Ptr frame =
      std::make_shared<gtsam_points::PointCloudCPU>(W_points);
    // Reset the map to a new map
    ivox_map_ = std::make_shared<IncrementalVoxelMapPCL>(*ivox_map_);
    ivox_map_->underlying()->insert(*frame);
    debug_msg_.t_insertion = sw.lapMs();

    map_poses_.push_back(T_W_Be);
    geometry_msgs::Pose p;
    convert(T_W_Be, p);
    keyframe_poses_.poses.push_back(p);

    if (pub_map_.getNumSubscribers()) {
      // Publish the updated map
      pcl::PointCloud<Point>::Ptr W_map = ivox_map_->getCloud();
      publishCloud(pub_map_, *W_map, config.world_frame, ts_);
    }

    keyframe_poses_.header.stamp.fromSec(ts_);
    pub_keyframe_poses_.publish(keyframe_poses_);
  }
  debug_msg_.t_update_map = sw.elapsedMs();
}

void Geometric::publishDebug()
{
  debug_msg_.header.stamp.fromSec(ts_);
  pub_debug_.publish(debug_msg_);
}

}  // namespace lidar
}  // namespace mimosa
