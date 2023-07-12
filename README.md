# docking_control
This project is for my Masters thesis topic: Cooperative Docking Control and Priority Scheduling for Multi-Robot Autonomy

The objective of this research is to demonstrate the feasibility of an autonomous docking controller for multiple robots with a singular charging station that allows each robot to charge without running out of power before reaching the dock by utilizing a priority scheduler that uses a fuzzy-logic based ranking and queuing system.

This repository contains the autonomous docking controller portion and simulation package of the project. The priority scheduler can be found in the priority_scheduler repository located at https://github.com/HadrienRoy/priority_scheduler.git.

Acknowledgements:
The Pupil Labs apriltag package was used in this research.
The docking state manager within the controller is inspired from https://github.com/Adlink-ROS/apriltag_docking.

## Prerequisites
Follow steps in the priority_scheduler repository to install correctly.

Follow steps as in instructed in https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup for the ROS2 Foxy.

## Quick Start
```sh
# Camera
sudo apt install ros-foxy-image-tools
sudo apt install ros-foxy-usb-cam
sudo apt install ros-foxy-compressed-image-transport
sudo apt install ros-foxy-slam-toolbox

# Gazebo
sudo apt install ros-foxy-gazebo-ros-pkgs

# AprilTag
sudo pip install python3-pip
pip install pupil-apriltags

# Nav2
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup

# Turtlebot3
sudo apt install ros-foxy-turtlebot3-cartographer
sudo apt install rox-foxy-turtlebot3-navigation2

# Other
sudo apt install ros-foxy-rviz2

mkdir docking_control_ws/src
cd ~/docking_control_ws/src
git clone https://github.com/HadrienRoy/docking_control.git
git clone https://github.com/swarmBots-ipa/tf_relay.git
git clone https://github.com/oKermorgant/map_simulator.git
cd ..
colcon build --symlink-install
```

## Usage
To run the simulation, perform the following steps:

Run the priority scheduler obtained from the priority_scheduler repo
```sh
ros2 launch docking_scheduler scheduler
```

### 2D Simulation: 
Run each of the following in separate terminals
```sh
ros2 launch sim_bringup simulation_2d.launch.py
ros2 launch sim_bringup spawn_tb3_0_2d.launch.py
ros2 launch sim_bringup spawn_tb3_1_2d.launch.py
ros2 launch sim_bringup spawn_tb3_2_2d.launch.py
```

### 3D Simulation: 
Can be performed in nested-loop style or single style.

Run tf_relay (number of robot can be changed within code) despite using nested loop or single style
```sh
ros2 launch tf_relay "tb3" 3
```
For nested-loop style:
```sh
ros2 launch sim_bringup multi_spawn_robot.launch.py
```

For single style: Run all the following commands in separate terminals
```sh
ros2 launch sim_bringup start_world.launch.py
ros2 launch sim_bringup spawn_tb3_0.launch.py
ros2 launch sim_bringup spawn_tb3_1.launch.py
ros2 launch sim_bringup spawn_tb3_2.launch.py
```
If only RViz is necessary, then do not launch start_world from above.



