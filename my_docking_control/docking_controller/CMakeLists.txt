cmake_minimum_required(VERSION 3.5)
project(docking_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(docking_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

# find_package(my_simulations REQUIRED)
include_directories(include)


set(CTRL_EXEC_NAME "controller")

set(DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
  "docking_interfaces"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav2_msgs"
  "rclcpp_action"
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/nav_to_pose.py
  DESTINATION lib/${PROJECT_NAME}
)



add_executable( docking_controller 
  src/controller.cpp 
  src/docking_server.cpp 
  src/queue_update_client.cpp
  src/state_update_server.cpp
  src/rank_update_server.cpp
  src/nav_send_goal.cpp
  )
ament_target_dependencies( docking_controller ${DEPENDENCIES} )

add_executable( docking_client src/docking_client.cpp )
ament_target_dependencies( docking_client ${DEPENDENCIES} )

# add_executable( nav_send_goal src/nav_send_goal.cpp )
# ament_target_dependencies( nav_send_goal ${DEPENDENCIES} )

# FOR TESTING
# add_executable( record_data src/record_data.cpp )
# ament_target_dependencies( record_data ${DEPENDENCIES} )

install(TARGETS
  docking_controller
  docking_client
  # nav_send_goal
  # record_data # TEST
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
