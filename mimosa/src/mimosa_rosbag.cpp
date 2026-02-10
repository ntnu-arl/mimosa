// Copyright (c) 2025, Autonomous Robots Lab, Norwegian University of Science and Technology
// All rights reserved.

// This source code is licensed under the BSD-style license found in the
// LICENSE file in the root directory of this source tree.

// IMU and Graph managers
#include "mimosa/graph/manager.hpp"
#include "mimosa/imu/manager.hpp"

// Exteroceptive sensor managers
#include "mimosa/lidar/manager.hpp"
#include "mimosa/radar/manager.hpp"
#include "mimosa/odometry/manager.hpp"

// ROS
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

// C++
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <filesystem>
#include <regex>

// Function to set terminal to non-blocking mode
void setNonBlocking(bool enable)
{
  struct termios ttystate;
  int fd = STDIN_FILENO;
  tcgetattr(fd, &ttystate);
  if (enable) {
    ttystate.c_lflag &= ~(ICANON | ECHO);  // Disable canonical mode and echo
    tcsetattr(fd, TCSANOW, &ttystate);
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  } else {
    ttystate.c_lflag |= ICANON | ECHO;  // Enable canonical mode and echo
    tcsetattr(fd, TCSANOW, &ttystate);
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
  }
}

int main(int argc, char ** argv)
{
  config::Settings().print_missing = true;

  ros::init(argc, argv, "mimosa_node");
  ros::NodeHandle pnh("~");
  ros::Publisher clock_pub = pnh.advertise<rosgraph_msgs::Clock>("/clock", 10);

  ros::param::set("/use_sim_time", true);

  auto imu_manager = std::make_shared<mimosa::imu::Manager>(pnh);
  auto graph_manager = std::make_shared<mimosa::graph::Manager>(pnh, imu_manager);

  // Exteroceptive sensor managers
  mimosa::lidar::Manager lidar_manager(pnh, imu_manager, graph_manager);
  mimosa::radar::Manager radar_manager(pnh, imu_manager, graph_manager);
  mimosa::odometry::Manager odometry_manager(pnh, imu_manager, graph_manager);

  // Read the bag name from parameter server
  std::string bag_name;
  if (!pnh.getParam("bag_name", bag_name)) {
    ROS_ERROR("Bag name not provided");
    return 1;
  }

  std::cout << "bag_name: " << bag_name << std::endl;

  // Vector to store matching bag paths
  std::vector<std::string> bag_paths;

  // Extract the directory and pattern from the bag_name
  std::string bag_dir = std::filesystem::path(bag_name).parent_path().string();
  if (bag_dir.empty()) {
    bag_dir = ".";
  }
  std::string bag_pattern = std::filesystem::path(bag_name).filename().string();

  // Convert the pattern to a regex
  std::string regex_pattern = std::regex_replace(bag_pattern, std::regex("\\*"), ".*");
  regex_pattern = std::regex_replace(regex_pattern, std::regex("\\?"), ".");

  std::cout << "bag_dir: " << bag_dir << std::endl;
  std::cout << "regex_pattern: " << regex_pattern << std::endl;

  // Iterate through files in the directory
  std::regex compiled_regex(regex_pattern);
  for (const auto & entry : std::filesystem::directory_iterator(bag_dir)) {
    if (
      entry.is_regular_file() &&
      std::regex_match(entry.path().filename().string(), compiled_regex)) {
      bag_paths.push_back(entry.path().string());
    }
  }

  // Sort the bag paths
  std::sort(bag_paths.begin(), bag_paths.end());

  float s_offset;
  if (!pnh.getParam("s", s_offset)) {
    s_offset = 0.0;
  }
  std::cout << "s_offset: " << s_offset << std::endl;

  float lidar_collection_delay = 0.113;  // s
  if (!pnh.getParam("lidar_collection_delay", lidar_collection_delay)) {
    lidar_collection_delay = 0.0;
  }
  std::queue<sensor_msgs::PointCloud2::ConstPtr> lidar_msg_queue;

  // Print out the bag_paths
  std::cout << "Opening these bags:" << std::endl;
  for (const auto & path : bag_paths) {
    std::cout << path << std::endl;
  }

  std::string imu_topic = imu_manager->getSubscribedTopic();
  std::string lidar_topic = lidar_manager.getSubscribedTopic();
  std::string radar_topic = radar_manager.getSubscribedTopic();
  std::string odometry_topic = odometry_manager.getSubscribedTopic();

  std::vector<std::string> topics = {imu_topic, lidar_topic, radar_topic, odometry_topic};

  std::cout << "Topics: " << std::endl;
  for (const auto & topic : topics) {
    std::cout << topic << std::endl;
  }

  // Set terminal to non-blocking mode
  setNonBlocking(true);

  bool paused = false;

  bool first = true;
  ros::Time start_time;

  for (const auto & path : bag_paths) {
    if (!ros::ok()) {
      break;
    }

    rosbag::Bag bag;
    std::cout << "Processing bag: " << path << std::endl;
    bag.open(path, rosbag::bagmode::Read);

    // Get the start time of the bags
    if (first) {
      start_time = rosbag::View(bag).getBeginTime();
      first = false;
    }

    // Iterate over the messages in the bag
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for (auto it = view.begin(); it != view.end();) {
      if (!ros::ok()) {
        break;
      }

      // Check for key press
      int ch = getchar();
      bool step = false;
      if (ch != EOF) {
        if (ch == ' ') {
          paused = !paused;
          // printf("Paused: %s\n", paused ? "Yes" : "No");
        } else if (paused && ch == 's') {
          // Step through messages when 's' is pressed
          // printf("Stepping\n");
          step = true;
        }
      }

      if (paused) {
        if (!step) {
          // When paused, wait briefly before checking again
          usleep(10000);  // Sleep for 10 milliseconds
          continue;
        }
      }

      auto & m = *it;

      ros::Time msg_time = m.getTime();
      if (msg_time - start_time < ros::Duration(s_offset)) {
        ++it;
        continue;
      }

      // Publish the time on the /clock topic
      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock = msg_time;
      clock_pub.publish(clock_msg);

      if (m.getTopic() == lidar_topic) {
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != nullptr) {
          if (lidar_collection_delay != 0) {
            // This message needs to be held back until the lidar collection delay has passed
            // Other messages should continue to be processed during this time
            lidar_msg_queue.push(msg);
          } else {
            lidar_manager.callback(msg);
          }
        }
      } else if (m.getTopic() == imu_topic) {
        sensor_msgs::Imu::ConstPtr msg = m.instantiate<sensor_msgs::Imu>();
        if (msg != nullptr) {
          imu_manager->callback(msg);

          if (lidar_msg_queue.size()) {
            if (
              msg->header.stamp - lidar_msg_queue.front()->header.stamp >
              ros::Duration(lidar_collection_delay)) {
              lidar_manager.callback(lidar_msg_queue.front());
              lidar_msg_queue.pop();
            }
          }
        }
      } else if (m.getTopic() == radar_topic) {
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg != nullptr) {
          radar_manager.callback(msg);
        }
      } else if (m.getTopic() == odometry_topic) {
        nav_msgs::Odometry::ConstPtr msg = m.instantiate<nav_msgs::Odometry>();
        if (msg != nullptr) {
          odometry_manager.callback(msg);
        }
      }
      ++it;
    }

    bag.close();
    std::cout << "Finished processing bag: " << path << std::endl;
  }

  // Restore terminal settings
  setNonBlocking(false);

  return 0;
}
