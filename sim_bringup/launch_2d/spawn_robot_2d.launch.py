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
    map_simulator_dir = get_package_share_directory('map_simulator')

    # Launch Configuration Variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace', default='tb3_1')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time',default='false')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    headless = LaunchConfiguration('headless')

    
    
    robot_name = LaunchConfiguration('robot_name', default='tb3_1')
    pose = {'x_pose': LaunchConfiguration('x_pose', default='0.00'),
            'y_pose': LaunchConfiguration('y_pose', default='0.00'),
            'z_pose': LaunchConfiguration('z_pose', default='0.00'),
            'roll': LaunchConfiguration('roll', default='0.00'),
            'pitch': LaunchConfiguration('pitch', default='0.00'),
            'yaw': LaunchConfiguration('yaw', default='0.00')}
    robot_prefix = LaunchConfiguration('robot_prefix')
    
    remappings = [
        # ('/tf', 'tf'),
        #           ('/tf_static', 'tf_static'),
                  ('map', '/map'), 
                  ('/scan', 'scan')]

    nav2_remappings = [
                  ('map', '/map'), 
                  ('/scan', 'scan')]    

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='tb3',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'tb3_1_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    # Declare launch arguments specific to simulation
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gzclient)')
    
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle_pi',
        description='name of the robot')

    declare_robot_prefix_cmd = DeclareLaunchArgument(
        'robot_prefix',
        default_value=[LaunchConfiguration('robot_name'),'/'])

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', "map.yaml"),
        description='Full path to map file to load.')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')



    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publiser_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'frame_prefix': robot_prefix,
            'use_sim_time': use_sim_time,
            'robot_description': robot_description
            }],
        remappings=remappings
        )

    # start_spawn_2d_cmd = Node(
    #     package='sim_bringup',
    #     namespace=namespace,
    #     executable="vel2joints.py",
    #     parameters=[{'static_tf': True,
    #                 'x_pose': pose['x_pose'], 
    #                 'y_pose': pose['y_pose'],
    #                 'yaw': pose['yaw']}]
    # )

    start_spawn_2d_cmd = Node (
        package='map_simulator', 
        namespace=namespace,
        executable='spawn', 
        parameters = [{  'zero_joints': False, 
                        'static_tf_odom': True, 
                        'radius': .22, 
                        'x': pose['x_pose'], 
                        'y': pose['y_pose'],
                        'theta': pose['yaw'],
                        'robot_color': [50,50,50], 
                        'laser_color': [0,255,0]}])

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
        # remappings=remappings
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
    start_rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file}.items())

    
    # Start nav2 pkg
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'map': map_yaml_file,
                          'slam': slam,
                          'use_sim_time': use_sim_time,
                          'autostart': autostart,
                          'params_file': params_file}.items(),
    )

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                                'use_sim_time': use_sim_time,
                                'autostart': autostart,
                                'params_file': params_file,
                                'use_lifecycle_mgr': 'false',
                                'map_subscribe_transient_local': 'true'}.items(),
        )
    ])

    lifecycle_nodes = [ 'controller_server',
                        'planner_server',
                        'recoveries_server',
                        'bt_navigator',
                        'waypoint_follower']

    navigation_group_cmd = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[params_file],
                remappings=remappings
                ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[params_file],
                remappings=remappings
                ),

            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                output='screen',
                parameters=[params_file],
                remappings=remappings
                ),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[params_file],
                remappings=remappings
                ),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[params_file],
                remappings=remappings
                ),

            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[
                            {'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]
                )

        
    ])
    

    

    # Create launch description
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_slam_cmd)

    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_prefix_cmd)

    # Add any actions
    ld.add_action(start_spawn_2d_cmd)
    
    # Add actions to launch naviagation nodes
    ld.add_action(start_robot_state_publiser_cmd)
    ld.add_action(start_rviz_cmd)

    # ld.add_action(start_nav2_cmd)

    # ld.add_action(bringup_cmd_group)

    ld.add_action(navigation_group_cmd)



    # Robot functions
    ld.add_action(start_gazebo_battery_state_cmd)
    ld.add_action(start_detect_tag_pupil_cmd)
    ld.add_action(start_docking_controller_cmd)
    ld.add_action(start_docking_client_cmd)

    return ld
