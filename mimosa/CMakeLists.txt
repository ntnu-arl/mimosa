cmake_minimum_required(VERSION 3.0.2)
project(mimosa)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

add_definitions(-DUSE_OPENMP=TRUE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(config_utilities REQUIRED)

find_package(PCL REQUIRED)
message("PCL Version:" ${PCL_VERSION})
message("PCL Path:" ${PCL_DIR})

find_package(OpenCV REQUIRED)
message("OpenCV Version:" ${OpenCV_VERSION})
message("OpenCV Path:" ${OpenCV_DIR})

find_package(GTSAM 4.2.0 REQUIRED)
message("GTSAM version: ${GTSAM_VERSION}")
message("GTSAM path: ${GTSAM_DIR}")

find_package(gtsam_points REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

find_package(OpenMP REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES mimosa_lib
 CATKIN_DEPENDS message_runtime nav_msgs roscpp sensor_msgs
 DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

set(SRC_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Define the folders for each sensor type
set(SRC_FOLDERS
    imu
    graph
    lidar
)

# Initialize an empty list to hold all source files
set(SOURCES)

# Loop over each sensor folder and add all .cpp files to the SOURCES list
foreach(FOLDER ${SRC_FOLDERS})
    file(GLOB_RECURSE SENSOR_SOURCES ${SRC_ROOT}/${FOLDER}/*.cpp)
    list(APPEND SOURCES ${SENSOR_SOURCES})
endforeach()

# Create a library from the collected source files
add_library(${PROJECT_NAME}_lib ${SOURCES})

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/mimosa.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/mimosa_node.cpp)
add_executable(${PROJECT_NAME}_rosbag src/mimosa_rosbag.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_rosbag ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  config_utilities::config_utilities
  ${PCL_LIBRARIES}
  gtsam
  gtsam_unstable
  gtsam_points
  ${OpenCV_LIBS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  config_utilities::config_utilities
  ${PCL_LIBRARIES}
  gtsam
  gtsam_unstable
  gtsam_points
  ${OpenCV_LIBS}
)

target_compile_options(${PROJECT_NAME}_node PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)

target_link_libraries(${PROJECT_NAME}_rosbag
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  config_utilities::config_utilities
  ${PCL_LIBRARIES}
  gtsam
  gtsam_unstable
  gtsam_points
  ${OpenCV_LIBS}
)

target_compile_options(${PROJECT_NAME}_rosbag PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mimosa.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
