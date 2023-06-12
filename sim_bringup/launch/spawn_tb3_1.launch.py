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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import yaml
from nav2_common.launch import RewrittenYaml, ReplaceString
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    bringup_dir = get_package_share_directory('sim_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Launch Configuration Variables
    use_sim_time = 'True'
    autostart = 'True'

    namespace = 'tb3_1'
    robot_name = 'tb3_1'
    robot_prefix = 'tb3_1/'

    pose = {'x_pose': -3.0,
            'y_pose': -3.0,
            'z_pose': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0}
    
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static')]

    
    params_file = os.path.join(bringup_dir, 'params', 'tb3_1_params.yaml'),
   
    param_substitutions = {'set_initial_pose': 'True',
                               'x': str(pose['x_pose']),
                               'y': str(pose['y_pose']),
                               'yaw': str(pose['yaw'])}

    configured_params = RewrittenYaml(
            source_file=params_file,
            # root_key=robot_name,
            param_rewrites=param_substitutions,
            convert_types=True
        )

    world = os.path.join(bringup_dir, 'worlds', 'wall_world.model')

    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
                         '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='screen')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], 
        output='screen'
        )


    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            # 'frame_prefix': robot_prefix,
            'use_sim_time': 'True',
            'robot_description': robot_description
            }],
        remappings=remappings
        )

    robot_sdf = os.path.join(bringup_dir, 'models', 'turtlebot3_waffle_pi', 'model.sdf')

    start_gazebo_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', namespace,
            '-x', str(pose['x_pose']), '-y', str(pose['y_pose']), '-z', str(pose['z_pose']),
            '-R', str(pose['roll']), '-P', str(pose['pitch']), '-Y', str(pose['yaw'])])

    ##########################################################################
    start_gazebo_battery_state_cmd = Node(
        package="sim_bringup",
        executable="gazebo_battery_state",
        name="gazebo_battery_state",
        namespace=namespace,
        parameters=[{'robot_id': robot_name}])


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
    ###########################################################################

    # Start rviz
    rviz_config_file = os.path.join(
            bringup_dir, 'rviz', 'nav2_namespaced_view.rviz')

    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': 'True',
                          'rviz_config': rviz_config_file}.items())

    # Start nav2 pkg
    map_yaml_file = os.path.join(bringup_dir, 'maps', "map.yaml")

    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': 'True',
                          'map': map_yaml_file,
                          'slam': 'False',
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': configured_params}.items(),
        # remappings=remappings
        )
    
    

    

    # Create launch description
    ld = LaunchDescription()

    # Declare launch options

    # Spawn robot
    ld.add_action(start_gazebo_server_cmd)
    # ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)

    ld.add_action(start_rviz_cmd)
    
    # Add actions to launch naviagation nodes
    ld.add_action(start_robot_state_publisher_cmd)

    # Start nav2
    ld.add_action(start_nav2_cmd)

    # Robot functions
    ld.add_action(start_gazebo_battery_state_cmd)
    ld.add_action(start_detect_tag_pupil_cmd)
    ld.add_action(start_docking_controller_cmd)
    ld.add_action(start_docking_client_cmd)

    return ld
