cmake_minimum_required(VERSION 3.5)
project(sim_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_common)
find_package(navigation2)

nav2_package()

find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

find_package(docking_interfaces REQUIRED)

find_package(urdf REQUIRED)



### BUILD ###
set(DEPENDENCIES
  "rclcpp"
  "sensor_msgs"
  "docking_interfaces"
  "geometry_msgs"
)


### INSTALL ###

install(DIRECTORY 
  launch
  maps
  models
  params
  rviz
  simple_launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  # launch/spawn_robot.py
  simple_launch/vel2joints.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install C++ modules
add_executable( gazebo_battery_state launch/gazebo_battery_state.cpp )
ament_target_dependencies( gazebo_battery_state ${DEPENDENCIES} )

add_executable( init_pose_publisher launch/init_pose_publisher.cpp )
ament_target_dependencies( init_pose_publisher ${DEPENDENCIES} )

install(TARGETS
  gazebo_battery_state
  init_pose_publisher
  DESTINATION lib/${PROJECT_NAME}
)




ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_export_dependencies(docking_interfaces)
ament_package()