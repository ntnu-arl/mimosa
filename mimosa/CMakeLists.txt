cmake_minimum_required(VERSION 3.0.2)
project(mimosa)

add_compile_options(-std=c++17)
add_definitions(-DUSE_OPENMP=1)

# ##############################################################################
# Dependencies ##
# ##############################################################################

find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             geometry_msgs
             ${PROJECT_NAME}_msgs
             nav_msgs
             pcl_conversions
             pcl_ros
             rosbag
             roscpp
             sensor_msgs
             std_msgs
             tf2_ros
             visualization_msgs
)

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

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  ${PROJECT_NAME}_msgs
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
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
    ${catkin_LIBRARIES}
    config_utilities::config_utilities
    ${PCL_LIBRARIES}
    gtsam
    gtsam_unstable
    gtsam_points::gtsam_points
    ${OpenCV_LIBS}
    spdlog::spdlog
)

# Strict compiler warnings for executables
set(STRICT_WARNINGS
    $<$<CXX_COMPILER_ID:MSVC>:/W4
    /WX>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall
    -Wextra
    -Wpedantic
    -Werror>
)

# Library
add_library(${PROJECT_NAME}_lib ${SOURCES})
add_dependencies(
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_options(
  ${PROJECT_NAME}_lib
  PRIVATE -pthread
          -fexceptions
          ${OpenMP_CXX_FLAGS}
)
target_link_libraries(${PROJECT_NAME}_lib ${COMMON_LIBS})
install(
  TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Executables
set(EXECUTABLES node rosbag)
foreach(EXEC ${EXECUTABLES})
  add_executable(${PROJECT_NAME}_${EXEC} src/${PROJECT_NAME}_${EXEC}.cpp)
  add_dependencies(
    ${PROJECT_NAME}_${EXEC}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
  )
  target_link_libraries(
    ${PROJECT_NAME}_${EXEC}
    ${PROJECT_NAME}_lib
    ${COMMON_LIBS}
  )
  target_compile_options(
    ${PROJECT_NAME}_${EXEC}
    PRIVATE ${STRICT_WARNINGS}
            -pthread
            -fexceptions
            ${OpenMP_CXX_FLAGS}
  )
  install(TARGETS ${PROJECT_NAME}_${EXEC}
          RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h*"
)

set(EXTRA_INSTALL_DIRS config launch)
foreach(directory IN LISTS ARGV)
  install(DIRECTORY ${directory}
          DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
endforeach()

set(SCRIPTS batch_process.sh evaluate.sh)
foreach(script IN LISTS SCRIPTS)
  install(PROGRAMS scripts/${script}
          DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
endforeach()
