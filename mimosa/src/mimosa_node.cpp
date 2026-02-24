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

// ROS
#include <ros/callback_queue.h>

// C++
#include <thread>

int main(int argc, char ** argv)
{
  config::Settings().print_missing = true;

  ros::init(argc, argv, "mimosa_node");
  ros::NodeHandle pnh("~");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // High priority node handle specifically for the IMU since it is at a very high frequency
  // compared to the other sensors
  ros::NodeHandle pnh_imu("~");
  ros::CallbackQueue callback_queue_imu;
  pnh_imu.setCallbackQueue(&callback_queue_imu);

  auto imu_manager = std::make_shared<mimosa::imu::Manager>(pnh_imu);
  auto graph_manager = std::make_shared<mimosa::graph::Manager>(pnh, imu_manager);

  std::thread imu_thread([&callback_queue_imu]() {
    ros::SingleThreadedSpinner spinner;
    spinner.spin(&callback_queue_imu);
  });

  // Exteroceptive sensor managers
  mimosa::lidar::Manager lidar_manager(pnh, imu_manager, graph_manager);
  mimosa::radar::Manager radar_manager(pnh, imu_manager, graph_manager);
  mimosa::odometry::Manager odometry_manager(pnh, imu_manager, graph_manager);

  ros::waitForShutdown();
  imu_thread.join();

  return 0;
}
