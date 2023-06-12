#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # Launch Configuration Variables
    namespace = LaunchConfiguration('namespace', default='tb3_1')
    robot_name = LaunchConfiguration('robot_name', default='tb3_1')

    pose = {'x_pose': LaunchConfiguration('x_pose', default='0.00'),
            'y_pose': LaunchConfiguration('y_pose', default='0.00')}

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='tb3',
        description='Top-level namespace')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle_pi',
        description='name of the robot')


    # Nodes
    start_detect_tag_pupil_cmd = Node(
        package="my_apriltag",
        executable="detect_tag_pupil",
        name="detect_tag_pupil",
        namespace=namespace,
        parameters=[{'robot_id': robot_name}],
        remappings=remappings
        )

    start_docking_controller_cmd = Node(
        package="docking_controller",
        executable="docking_controller",
        name="docking_controller",
        namespace=namespace,
        parameters=[{'robot_id': robot_name,
                     'initial_approach_distance_tolerance': 0.4,
                     'final_approach_distance_tolerance': 0.2,
                     'init_x_pose': pose['x_pose'],
                     'init_y_pose': pose['y_pose'],
                     }])

    start_docking_client_cmd = Node(
        package="docking_controller",
        executable="docking_client",
        name="docking_client",
        namespace=namespace,
        parameters=[{'robot_id': robot_name}])


    # Create launch description
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)

    # Robot functions
    ld.add_action(start_detect_tag_pupil_cmd)
    ld.add_action(start_docking_controller_cmd)
    ld.add_action(start_docking_client_cmd)

    return ld
