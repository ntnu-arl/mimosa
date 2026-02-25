# Detect ROS version: prefer environment variable, fallback to roscpp/rclcpp
# detection
if(DEFINED ENV{ROS_VERSION})
  set(DETECTED_ROS_VERSION $ENV{ROS_VERSION})
  message(
    STATUS
      "Detected DETECTED_ROS_VERSION=${DETECTED_ROS_VERSION} from environment variable"
  )
else()
  message(
    STATUS
      "DETECTED_ROS_VERSION environment variable not set, detecting from packages..."
  )
  find_package(roscpp QUIET)
  if(roscpp_FOUND)
    set(DETECTED_ROS_VERSION 1)
  else()
    find_package(rclcpp QUIET)
    if(rclcpp_FOUND)
      set(DETECTED_ROS_VERSION 2)
    else()
      message(
        FATAL_ERROR
          "Could not detect ROS version. Neither ROS_VERSION env var is set nor roscpp/rclcpp packages found."
      )
    endif()
  endif()
endif()

if(NOT
   DETECTED_ROS_VERSION
   EQUAL
   1
   AND NOT
       DETECTED_ROS_VERSION
       EQUAL
       2
)
  message(
    FATAL_ERROR
      "Unsupported DETECTED_ROS_VERSION: ${DETECTED_ROS_VERSION}. Must be 1 or 2."
  )
endif()

message(WARNING "Building ${PROJECT_NAME} with ROS${DETECTED_ROS_VERSION}")
add_definitions(-DDETECTED_ROS_VERSION=${DETECTED_ROS_VERSION})
