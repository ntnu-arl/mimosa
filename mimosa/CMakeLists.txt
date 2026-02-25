cmake_minimum_required(VERSION 3.8)
project(mimosa)

add_compile_options(-std=c++17)
add_definitions(-DUSE_OPENMP=1)

list(
  APPEND
  CMAKE_MODULE_PATH
  "${CMAKE_CURRENT_SOURCE_DIR}/../cmake"
)
include(DetectROSVersion)

# ##############################################################################
# Dependencies ##
# ##############################################################################

# ROS
set(COMMON_ROS_DEPS
    cv_bridge
    geometry_msgs
    ${PROJECT_NAME}_msgs
    nav_msgs
    pcl_conversions
    pcl_ros
    sensor_msgs
    std_msgs
    tf2_ros
    visualization_msgs
)

# ROS-specific configuration
if(DETECTED_ROS_VERSION
   EQUAL
   1
)
  find_package(
    catkin REQUIRED
    COMPONENTS ${COMMON_ROS_DEPS}
               roscpp
               rosbag
  )
  catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}_lib
    CATKIN_DEPENDS
    message_runtime
    ${COMMON_ROS_DEPS}
  )
  set(ROS_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
  set(ROS_LIBRARIES ${catkin_LIBRARIES})
  set(ARCHIVE_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  set(LIBRARY_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
  set(RUNTIME_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(INCLUDE_DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  set(SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rosbag2_cpp REQUIRED)

  foreach(DEP ${COMMON_ROS_DEPS})
    find_package(${DEP} REQUIRED)
  endforeach()

  set(ROS2_DEPENDENCIES
      rclcpp
      rosbag2_cpp
      ${COMMON_ROS_DEPS}
  )
  set(ROS_INCLUDE_DIRS "")
  set(ROS_LIBRARIES "")
  set(ARCHIVE_DESTINATION lib)
  set(LIBRARY_DESTINATION lib)
  set(RUNTIME_DESTINATION bin)
  set(INCLUDE_DESTINATION include/${PROJECT_NAME})
  set(SHARE_DESTINATION share/${PROJECT_NAME})
endif()

# Non-ROS dependencies
find_package(config_utilities REQUIRED)
find_package(OpenCV REQUIRED)
find_package(
  GTSAM
  4.2.0
  REQUIRED
)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(gtsam_points REQUIRED)
find_package(OpenMP REQUIRED)
find_package(spdlog REQUIRED)

# Print dependency info
message(STATUS "PCL Version: ${PCL_VERSION}, Path: ${PCL_DIR}")
message(STATUS "OpenCV Version: ${OpenCV_VERSION}, Path: ${OpenCV_DIR}")
message(STATUS "GTSAM Version: ${GTSAM_VERSION}, Path: ${GTSAM_DIR}")
message(STATUS "gtsam_points Path: ${gtsam_points_DIR}")

include_directories(
  include
  SYSTEM
  ${ROS_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

# Collect all source files from sensor folders
set(SRC_CPP_FOLDERS
    imu
    graph
    lidar
    radar
    odometry
)
set(SOURCES)
foreach(FOLDER ${SRC_CPP_FOLDERS})
  file(
    GLOB_RECURSE
    SRC_CPP_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/${FOLDER}/*.cpp
  )
  list(
    APPEND
    SOURCES
    ${SRC_CPP_FILES}
  )
endforeach()

# Common libraries used by all targets
set(COMMON_LIBS
    ${ROS_LIBRARIES}
    config_utilities::config_utilities
    ${PCL_LIBRARIES}
    gtsam
    gtsam_unstable
    gtsam_points::gtsam_points
    ${OpenCV_LIBS}
    spdlog::spdlog
)

# Helper function to configure targets for ROS version
function(configure_ros_target TARGET_NAME)
  if(DETECTED_ROS_VERSION
     EQUAL
     1
  )
    add_dependencies(
      ${TARGET_NAME}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
  else()
    ament_target_dependencies(${TARGET_NAME} ${ROS2_DEPENDENCIES})
  endif()

  target_compile_features(
    ${TARGET_NAME} PUBLIC c_std_99 cxx_std_17
  ) # Require C99 and C++17
  target_compile_options(
    ${TARGET_NAME}
    PRIVATE $<$<CXX_COMPILER_ID:MSVC>:/W4
            /WX>
            $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall
            -Wextra
            -Wpedantic
            -Werror>
            -pthread
            -fexceptions
            ${OpenMP_CXX_FLAGS}
  )
  target_compile_definitions(
    ${TARGET_NAME} PRIVATE DETECTED_ROS_VERSION=${DETECTED_ROS_VERSION}
  )
  target_link_libraries(${TARGET_NAME} ${COMMON_LIBS})
endfunction()

# Library
add_library(${PROJECT_NAME}_lib ${SOURCES})
add_library(
  ${PROJECT_NAME}::${PROJECT_NAME}_lib
  ALIAS
  ${PROJECT_NAME}_lib
)
configure_ros_target(${PROJECT_NAME}_lib)
install(
  TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
  LIBRARY DESTINATION ${LIBRARY_DESTINATION}
  RUNTIME DESTINATION ${RUNTIME_DESTINATION}
)

# Executables
set(EXECUTABLES node rosbag)
foreach(EXEC ${EXECUTABLES})
  add_executable(${PROJECT_NAME}_${EXEC} src/${PROJECT_NAME}_${EXEC}.cpp)
  configure_ros_target(${PROJECT_NAME}_${EXEC})
  target_link_libraries(${PROJECT_NAME}_${EXEC} ${PROJECT_NAME}_lib)
  install(TARGETS ${PROJECT_NAME}_${EXEC}
          RUNTIME DESTINATION ${RUNTIME_DESTINATION}
  )
endforeach()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h*"
)
install(DIRECTORY config launch DESTINATION ${SHARE_DESTINATION})

install(PROGRAMS scripts/batch_process.sh scripts/evaluate.sh
        DESTINATION ${RUNTIME_DESTINATION}
)
