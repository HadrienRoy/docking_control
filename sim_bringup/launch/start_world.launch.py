# Author: Addison Sears-Collins
# Date: September 23, 2021
# Description: Load a world file into Gazebo.
# https://automaticaddison.com

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']


def generate_launch_description():
   
    
    bringup_dir = get_package_share_directory('sim_bringup')
    world = os.path.join(bringup_dir, 'worlds', 'wall_world.model')

    
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world],
        output='screen')


    # Create the launch description and populate
    ld = LaunchDescription()


    # Add any actions
    ld.add_action(start_gazebo_cmd)
   

    return ld
