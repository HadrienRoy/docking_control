cmake_minimum_required(VERSION 3.5)
project(docking_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/ChargingQueue.msg"
  "msg/CurrentState.msg"
  "msg/StartDocking.msg"
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
)

set(srv_files
  "srv/Docking.srv"
  "srv/StartAprilTagDetection.srv"
  "srv/GazeboChargeBattery.srv"
  "srv/QueueUpdate.srv"
  "srv/StateUpdate.srv"
  "srv/RankUpdate.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}

  DEPENDENCIES geometry_msgs std_msgs
)

ament_package()
