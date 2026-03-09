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
#if DETECTED_ROS_VERSION == 1
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#else
#include <spdlog/spdlog.h>

#include <rosbag2_cpp/reader.hpp>
#endif

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

  // ROS initialization and parameter setup
#if DETECTED_ROS_VERSION == 1
  ros::init(argc, argv, "mimosa_node");
  auto pnh = std::make_shared<ros::NodeHandle>("~");
  std::string config_path;
  pnh->param<std::string>("config_path", config_path, "config/mimosa.yaml");
  ros::Publisher clock_pub = pnh->advertise<rosgraph_msgs::Clock>("/clock", 10);
  ros::param::set("/use_sim_time", true);
  mimosa::ri::NodeHandle nh = pnh;
#else
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "mimosa_node",
    rclcpp::NodeOptions().parameter_overrides({rclcpp::Parameter("use_sim_time", true)}));
  std::string config_path =
    node->declare_parameter<std::string>("config_path", "config/mimosa.yaml");
  auto clock_pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  mimosa::ri::NodeHandle nh = node;
#endif

  auto imu_manager = std::make_shared<mimosa::imu::Manager>(config_path, nh);
  auto graph_manager = std::make_shared<mimosa::graph::Manager>(config_path, nh, imu_manager);

  // Exteroceptive sensor managers
  mimosa::lidar::Manager lidar_manager(config_path, nh, imu_manager, graph_manager);
  mimosa::radar::Manager radar_manager(config_path, nh, imu_manager, graph_manager);
  mimosa::odometry::Manager odometry_manager(config_path, nh, imu_manager, graph_manager);

  // Read the bag name from parameter server
  std::string bag_name;
#if DETECTED_ROS_VERSION == 1
  if (!pnh->getParam("bag_name", bag_name)) {
    ROS_ERROR("Bag name not provided");
    return 1;
  }
#else
  bag_name = node->declare_parameter<std::string>("bag_name", "");
  if (bag_name.empty()) {
    spdlog::error("Bag name not provided");
    return 1;
  }
#endif

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
  for (const auto & entry : std::filesystem::directory_iterator(bag_dir)) {
#if DETECTED_ROS_VERSION == 1
    bool is_bag_entry = entry.is_regular_file();
#else
    // ROS2 bags are directories containing metadata.yaml and .db3 files
    bool is_bag_entry = entry.is_regular_file() || entry.is_directory();
#endif
    if (
      is_bag_entry &&
      std::regex_match(entry.path().filename().string(), std::regex(regex_pattern))) {
      bag_paths.push_back(entry.path().string());
    }
  }

  // Sort the bag paths
  std::sort(bag_paths.begin(), bag_paths.end());

  float s_offset;
  float lidar_collection_delay;
#if DETECTED_ROS_VERSION == 1
  if (!pnh->getParam("s", s_offset)) {
    s_offset = 0.0;
  }
  lidar_collection_delay = 0.113;  // s
  if (!pnh->getParam("lidar_collection_delay", lidar_collection_delay)) {
    lidar_collection_delay = 0.0;
  }
#else
  s_offset = static_cast<float>(node->declare_parameter<double>("s", 0.0));
  lidar_collection_delay =
    static_cast<float>(node->declare_parameter<double>("lidar_collection_delay", 0.0));
#endif
  std::cout << "s_offset: " << s_offset << std::endl;

  std::queue<mimosa::ri::ConstSharedPtr<mimosa::ri::SensorMsgsPointCloud2>> lidar_msg_queue;

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

  // --- Bag reading loop ---
#if DETECTED_ROS_VERSION == 1
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
        } else if (paused && ch == 's') {
          step = true;
        }
      }

      if (paused) {
        if (!step) {
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

#else  // ROS2
  int64_t start_time_ns = 0;

  for (const auto & path : bag_paths) {
    if (!rclcpp::ok()) {
      break;
    }

    rosbag2_cpp::Reader reader;
    std::cout << "Processing bag: " << path << std::endl;
    reader.open(path);

    // Get the start time of the bags
    if (first) {
      auto metadata = reader.get_metadata();
      start_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        metadata.starting_time.time_since_epoch())
                        .count();
      first = false;
    }

    rosbag2_storage::StorageFilter filter;
    filter.topics = topics;
    reader.set_filter(filter);

    while (reader.has_next()) {
      if (!rclcpp::ok()) {
        break;
      }

      // Check for key press
      int ch = getchar();
      bool step = false;
      if (ch != EOF) {
        if (ch == ' ') {
          paused = !paused;
        } else if (paused && ch == 's') {
          step = true;
        }
      }

      if (paused) {
        if (!step) {
          usleep(10000);  // Sleep for 10 milliseconds
          continue;
        }
      }

      auto bag_msg = reader.read_next();
      int64_t msg_time_ns = bag_msg->recv_timestamp;

      if ((msg_time_ns - start_time_ns) < static_cast<int64_t>(s_offset * 1e9)) {
        continue;
      }

      // Publish the time on the /clock topic
      rosgraph_msgs::msg::Clock clock_msg;
      clock_msg.clock.sec = static_cast<int32_t>(msg_time_ns / 1000000000LL);
      clock_msg.clock.nanosec = static_cast<uint32_t>(msg_time_ns % 1000000000LL);
      clock_pub->publish(clock_msg);

      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);

      if (bag_msg->topic_name == lidar_topic) {
        auto msg = std::make_shared<mimosa::ri::SensorMsgsPointCloud2>();
        rclcpp::Serialization<mimosa::ri::SensorMsgsPointCloud2> serializer;
        serializer.deserialize_message(&serialized_msg, msg.get());
        if (lidar_collection_delay != 0) {
          lidar_msg_queue.push(msg);
        } else {
          lidar_manager.callback(msg);
        }
      } else if (bag_msg->topic_name == imu_topic) {
        auto msg = std::make_shared<mimosa::ri::SensorMsgsImu>();
        rclcpp::Serialization<mimosa::ri::SensorMsgsImu> serializer;
        serializer.deserialize_message(&serialized_msg, msg.get());
        imu_manager->callback(msg);

        if (!lidar_msg_queue.empty()) {
          rclcpp::Time imu_stamp(msg->header.stamp);
          rclcpp::Time lidar_stamp(lidar_msg_queue.front()->header.stamp);
          if (imu_stamp - lidar_stamp > rclcpp::Duration::from_seconds(lidar_collection_delay)) {
            lidar_manager.callback(lidar_msg_queue.front());
            lidar_msg_queue.pop();
          }
        }
      } else if (bag_msg->topic_name == radar_topic) {
        auto msg = std::make_shared<mimosa::ri::SensorMsgsPointCloud2>();
        rclcpp::Serialization<mimosa::ri::SensorMsgsPointCloud2> serializer;
        serializer.deserialize_message(&serialized_msg, msg.get());
        radar_manager.callback(msg);
      } else if (bag_msg->topic_name == odometry_topic) {
        auto msg = std::make_shared<mimosa::ri::NavMsgsOdometry>();
        rclcpp::Serialization<mimosa::ri::NavMsgsOdometry> serializer;
        serializer.deserialize_message(&serialized_msg, msg.get());
        odometry_manager.callback(msg);
      }
    }

    std::cout << "Finished processing bag: " << path << std::endl;
  }

  rclcpp::shutdown();
#endif

  // Restore terminal settings
  setNonBlocking(false);

  return 0;
}
