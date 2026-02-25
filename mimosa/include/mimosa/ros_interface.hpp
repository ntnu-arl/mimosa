// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

// ROS version-specific includes
#if DETECTED_ROS_VERSION == 1
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mimosa_msgs/GraphManagerDebug.h>
#include <mimosa_msgs/ImuManagerDebug.h>
#include <mimosa_msgs/LidarGeometricDebug.h>
#include <mimosa_msgs/LidarManagerDebug.h>
#include <mimosa_msgs/LidarPhotometricDebug.h>
#include <mimosa_msgs/RadarManagerDebug.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#else

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mimosa_msgs/msg/graph_manager_debug.hpp>
#include <mimosa_msgs/msg/imu_manager_debug.hpp>
#include <mimosa_msgs/msg/lidar_geometric_debug.hpp>
#include <mimosa_msgs/msg/lidar_manager_debug.hpp>
#include <mimosa_msgs/msg/lidar_photometric_debug.hpp>
#include <mimosa_msgs/msg/radar_manager_debug.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#endif

namespace mimosa
{
namespace ros_interface
{

#if DETECTED_ROS_VERSION == 1
// ROS1 type aliases
using NodeHandle = std::shared_ptr<ros::NodeHandle>;
using Time = ros::Time;
using Duration = ros::Duration;
using Rate = ros::Rate;

// Publisher type aliases
template <typename MessageType>
using Publisher = std::shared_ptr<ros::Publisher>;

template <typename MessageType>
using Subscriber = std::shared_ptr<ros::Subscriber>;

// Message type aliases
using StdMsgsHeader = std_msgs::Header;
using SensorMsgsImu = sensor_msgs::Imu;
using SensorMsgsPointField = sensor_msgs::PointField;
using SensorMsgsPointCloud2 = sensor_msgs::PointCloud2;
using SensorMsgsImage = sensor_msgs::Image;
using GeometryMsgsPoint = geometry_msgs::Point;
using GeometryMsgsVector3 = geometry_msgs::Vector3;
using GeometryMsgsQuaternion = geometry_msgs::Quaternion;
using GeometryMsgsTransform = geometry_msgs::Transform;
using GeometryMsgsTransformStamped = geometry_msgs::TransformStamped;
using GeometryMsgsPose = geometry_msgs::Pose;
using GeometryMsgsPoseStamped = geometry_msgs::PoseStamped;
using GeometryMsgsPoseArray = geometry_msgs::PoseArray;
using NavMsgsOdometry = nav_msgs::Odometry;
using NavMsgsPath = nav_msgs::Path;
using VisualizationMsgsMarker = visualization_msgs::Marker;
using VisualizationMsgsMarkerArray = visualization_msgs::MarkerArray;
using MimosaMsgsImuManagerDebug = mimosa_msgs::ImuManagerDebug;
using MimosaMsgsRadarManagerDebug = mimosa_msgs::RadarManagerDebug;
using MimosaMsgsGraphManagerDebug = mimosa_msgs::GraphManagerDebug;
using MimosaMsgsLidarPhotometricDebug = mimosa_msgs::LidarPhotometricDebug;
using MimosaMsgsLidarGeometricDebug = mimosa_msgs::LidarGeometricDebug;
using MimosaMsgsLidarManagerDebug = mimosa_msgs::LidarManagerDebug;

// Ptr and ConstPtr were deprecated in ROS2. The naming is kept wrt to ROS2 but the
// using declarations here allow the same code to compile in both ROS1 and ROS2.
template <typename MessageType>
using SharedPtr = typename MessageType::Ptr;
template <typename MessageType>
using ConstSharedPtr = typename MessageType::ConstPtr;

// Transform broadcasters
using TransformBroadcaster = std::shared_ptr<tf2_ros::TransformBroadcaster>;
using StaticTransformBroadcaster = std::shared_ptr<tf2_ros::StaticTransformBroadcaster>;
inline StaticTransformBroadcaster create_static_transform_broadcaster(NodeHandle)
{
  return std::make_shared<tf2_ros::StaticTransformBroadcaster>();
}
inline TransformBroadcaster create_transform_broadcaster(NodeHandle)
{
  return std::make_shared<tf2_ros::TransformBroadcaster>();
}

// Utility functions
template <typename MessageType>
Publisher<MessageType> create_publisher(
  NodeHandle & nh, const std::string & topic, int queue_size, bool latch = false)
{
  return std::make_shared<ros::Publisher>(nh->advertise<MessageType>(topic, queue_size, latch));
}

template <typename MessageType>
Subscriber<MessageType> create_subscriber(
  NodeHandle & nh, const std::string & topic, int queue_size,
  std::function<void(const typename MessageType::ConstPtr &)> callback)
{
  return std::make_shared<ros::Subscriber>(nh->subscribe<MessageType>(topic, queue_size, callback));
}

inline Time now() { return ros::Time::now(); }
inline void spin_once() { ros::spinOnce(); }
inline bool ok() { return ros::ok(); }

// Time conversion utilities
inline double to_seconds(const Time & time) { return time.toSec(); }
inline void from_seconds(Time & time, double secs) { time.fromSec(secs); }

// Publisher utilities
inline size_t get_num_subscribers(const std::shared_ptr<ros::Publisher> & pub)
{
  return pub->getNumSubscribers();
}

inline const std::string PrivateTopicPrefix = std::string("");

#else
// ROS2 type aliases
using NodeHandle = std::shared_ptr<rclcpp::Node>;
using Time = rclcpp::Time;
using Duration = rclcpp::Duration;
using Rate = rclcpp::Rate;

// Publisher type aliases
template <typename MessageType>
using Publisher = std::shared_ptr<rclcpp::Publisher<MessageType> >;

template <typename MessageType>
using Subscriber = std::shared_ptr<rclcpp::Subscription<MessageType> >;

// Message type aliases
using StdMsgsHeader = std_msgs::msg::Header;
using SensorMsgsImu = sensor_msgs::msg::Imu;
using SensorMsgsPointField = sensor_msgs::msg::PointField;
using SensorMsgsPointCloud2 = sensor_msgs::msg::PointCloud2;
using SensorMsgsImage = sensor_msgs::msg::Image;
using GeometryMsgsPoint = geometry_msgs::msg::Point;
using GeometryMsgsVector3 = geometry_msgs::msg::Vector3;
using GeometryMsgsQuaternion = geometry_msgs::msg::Quaternion;
using GeometryMsgsTransform = geometry_msgs::msg::Transform;
using GeometryMsgsTransformStamped = geometry_msgs::msg::TransformStamped;
using GeometryMsgsPose = geometry_msgs::msg::Pose;
using GeometryMsgsPoseStamped = geometry_msgs::msg::PoseStamped;
using GeometryMsgsPoseArray = geometry_msgs::msg::PoseArray;
using NavMsgsOdometry = nav_msgs::msg::Odometry;
using NavMsgsPath = nav_msgs::msg::Path;
using VisualizationMsgsMarker = visualization_msgs::msg::Marker;
using VisualizationMsgsMarkerArray = visualization_msgs::msg::MarkerArray;
using MimosaMsgsImuManagerDebug = mimosa_msgs::msg::ImuManagerDebug;
using MimosaMsgsRadarManagerDebug = mimosa_msgs::msg::RadarManagerDebug;
using MimosaMsgsGraphManagerDebug = mimosa_msgs::msg::GraphManagerDebug;
using MimosaMsgsLidarPhotometricDebug = mimosa_msgs::msg::LidarPhotometricDebug;
using MimosaMsgsLidarGeometricDebug = mimosa_msgs::msg::LidarGeometricDebug;
using MimosaMsgsLidarManagerDebug = mimosa_msgs::msg::LidarManagerDebug;

// Ptr and ConstPtr were deprecated in ROS2. The naming is kept wrt to ROS2 but the
// using declarations here allow the same code to compile in both ROS1 and ROS2.
template <typename MessageType>
using SharedPtr = typename MessageType::SharedPtr;
template <typename MessageType>
using ConstSharedPtr = typename MessageType::ConstSharedPtr;

// Transform broadcasters
using TransformBroadcaster = std::shared_ptr<tf2_ros::TransformBroadcaster>;
using StaticTransformBroadcaster = std::shared_ptr<tf2_ros::StaticTransformBroadcaster>;
inline StaticTransformBroadcaster create_static_transform_broadcaster(NodeHandle & nh)
{
  return std::make_shared<tf2_ros::StaticTransformBroadcaster>(nh);
}
inline TransformBroadcaster create_transform_broadcaster(NodeHandle & nh)
{
  return std::make_shared<tf2_ros::TransformBroadcaster>(nh);
}

// Utility functions
template <typename MessageType>
Publisher<MessageType> create_publisher(
  NodeHandle & node, const std::string & topic, int queue_size, bool latch = false)
{
  rclcpp::QoS qos(queue_size);
  if (latch) {
    qos = qos.transient_local();
  }
  return node->template create_publisher<MessageType>(topic, qos);
}

template <typename MessageType>
Subscriber<MessageType> create_subscriber(
  NodeHandle & node, const std::string & topic, int queue_size,
  std::function<void(const std::shared_ptr<MessageType>)> callback)
{
  rclcpp::QoS qos(queue_size);
  return node->template create_subscription<MessageType>(topic, qos, callback);
}

inline Time now(NodeHandle & node) { return node->now(); }
inline void spin_once(NodeHandle & node) { rclcpp::spin_some(node); }
inline bool ok() { return rclcpp::ok(); }

// Time conversion utilities
inline double to_seconds(const Time & time) { return time.seconds(); }
inline void from_seconds(builtin_interfaces::msg::Time & time, double secs)
{
  time.sec = static_cast<int32_t>(secs);
  time.nanosec = static_cast<uint32_t>((secs - time.sec) * 1e9);
}

// Publisher utilities
template <typename MessageType>
inline size_t get_num_subscribers(const Publisher<MessageType> & pub)
{
  return pub->get_subscription_count();
}

inline const std::string PrivateTopicPrefix = std::string("~/");

#endif

}  // namespace ros_interface

namespace ri = ros_interface;
}  // namespace mimosa
