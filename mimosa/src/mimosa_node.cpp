// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

// IMU and Graph managers
#include "mimosa/graph/manager.hpp"
#include "mimosa/imu/manager.hpp"

// Exteroceptive sensor managers
#include "mimosa/lidar/manager.hpp"
#include "mimosa/odometry/manager.hpp"
#include "mimosa/radar/manager.hpp"

// C++
#include <thread>

#if DETECTED_ROS_VERSION == 1
#include <ros/callback_queue.h>
#endif

int main(int argc, char ** argv)
{
  config::Settings().print_missing = true;

#if DETECTED_ROS_VERSION == 1
  // ROS1: init, spinners, and callback queue for IMU
  ros::init(argc, argv, "mimosa_node");
  auto pnh = std::make_shared<ros::NodeHandle>("~");
  std::string config_path;
  pnh->param<std::string>("config_path", config_path, "config/mimosa.yaml");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // High priority node handle specifically for the IMU since it is at a very high frequency
  // compared to the other sensors
  auto pnh_imu = std::make_shared<ros::NodeHandle>("~");
  ros::CallbackQueue callback_queue_imu;
  pnh_imu->setCallbackQueue(&callback_queue_imu);

  mimosa::ri::NodeHandle nh = pnh;
  mimosa::ri::NodeHandle nh_imu = pnh_imu;

  auto imu_manager = std::make_shared<mimosa::imu::Manager>(config_path, nh_imu);
  auto graph_manager = std::make_shared<mimosa::graph::Manager>(config_path, nh, imu_manager);

  std::thread imu_thread([&callback_queue_imu]() {
    ros::SingleThreadedSpinner spinner;
    spinner.spin(&callback_queue_imu);
  });

  // Exteroceptive sensor managers
  mimosa::lidar::Manager lidar_manager(config_path, nh, imu_manager, graph_manager);
  mimosa::radar::Manager radar_manager(config_path, nh, imu_manager, graph_manager);
  mimosa::odometry::Manager odometry_manager(config_path, nh, imu_manager, graph_manager);

  ros::waitForShutdown();
  imu_thread.join();

#else
  // ROS2: init, multi-threaded executor, and dedicated callback group for IMU
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mimosa_node");

  node->declare_parameter("config_path", "config/mimosa.yaml");
  std::string config_path = node->get_parameter("config_path").as_string();

  mimosa::ri::NodeHandle nh = node;

  // Create a dedicated callback group for IMU so it is never blocked by other callbacks
  auto imu_cb_group = mimosa::ri::create_callback_group(nh);

  auto imu_manager = std::make_shared<mimosa::imu::Manager>(config_path, nh, imu_cb_group);
  auto graph_manager = std::make_shared<mimosa::graph::Manager>(config_path, nh, imu_manager);

  // Exteroceptive sensor managers
  mimosa::lidar::Manager lidar_manager(config_path, nh, imu_manager, graph_manager);
  mimosa::radar::Manager radar_manager(config_path, nh, imu_manager, graph_manager);
  mimosa::odometry::Manager odometry_manager(config_path, nh, imu_manager, graph_manager);

  // Use a multi-threaded executor so callbacks on different topics run in parallel,
  // and the IMU callback group gets its own thread
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

#endif

  return 0;
}
