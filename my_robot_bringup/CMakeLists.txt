cmake_minimum_required(VERSION 3.5)
project(my_robot_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(nav2_common)
find_package(navigation2)
find_package(rclcpp REQUIRED)
nav2_package()

install(DIRECTORY 
  launch
  camera_info
  params
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
