cmake_minimum_required(VERSION 3.0.2)
project(mimosa)

add_compile_options(-std=c++17)
add_definitions(-DUSE_OPENMP=TRUE)

#################
## Dependencies ##
#################

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  pcl_conversions
  tf2_ros
  message_generation
  geometry_msgs
  cv_bridge
  visualization_msgs
  rosbag
  mimosa_msgs
)

find_package(config_utilities REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GTSAM 4.2.0 REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(gtsam_points REQUIRED)
find_package(OpenMP REQUIRED)

# Print dependency info
message(STATUS "PCL Version: ${PCL_VERSION}, Path: ${PCL_DIR}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}, Path: ${OpenCV_DIR}")
message(STATUS "GTSAM Version: ${GTSAM_VERSION}, Path: ${GTSAM_DIR}")
message(STATUS "gtsam_points Path: ${gtsam_points_DIR}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS message_runtime nav_msgs roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Collect all source files from sensor folders
set(SRC_CPP_FOLDERS imu graph lidar radar odometry)
set(SOURCES)
foreach(FOLDER ${SRC_CPP_FOLDERS})
  file(GLOB_RECURSE SRC_CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/${FOLDER}/*.cpp)
  list(APPEND SOURCES ${SRC_CPP_FILES})
endforeach()

# Common libraries used by all targets
set(COMMON_LIBS
  ${catkin_LIBRARIES}
  config_utilities::config_utilities
  ${PCL_LIBRARIES}
  gtsam
  gtsam_unstable
  gtsam_points::gtsam_points
  ${OpenCV_LIBS}
)

# Strict compiler warnings for executables
set(STRICT_WARNINGS
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)

# Library
add_library(${PROJECT_NAME}_lib ${SOURCES})
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${COMMON_LIBS})

# Executables
set(EXECUTABLES node rosbag)
foreach(EXEC ${EXECUTABLES})
  add_executable(${PROJECT_NAME}_${EXEC} src/${PROJECT_NAME}_${EXEC}.cpp)
  add_dependencies(${PROJECT_NAME}_${EXEC} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(${PROJECT_NAME}_${EXEC} ${PROJECT_NAME}_lib ${COMMON_LIBS})
  target_compile_options(${PROJECT_NAME}_${EXEC} PRIVATE ${STRICT_WARNINGS})
endforeach()
