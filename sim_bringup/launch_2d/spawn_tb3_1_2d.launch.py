#!/usr/bin/env python3

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

    # Launch Configuration Variables
    use_sim_time = True
    namespace = 'tb3_1'

    autostart = True

    robot_name = 'tb3_1'
    robot_prefix = 'tb3_1/'

    pose = {'x_pose': -7.0,
            'y_pose': -5.0,
            'z_pose': 0.0,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0}
    
    remappings = [
                  ('map', '/map'), 
                  ('/scan', 'scan')]

    # Declare launch arguments
    params_file = os.path.join(bringup_dir, 'params', 'waffle_pi.yaml'),
   
    bt_xml_filename = os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    param_substitutions = {
                                'use_sim_time': 'True',
                                'autostart': 'True',
                                'default_bt_xml_filename': bt_xml_filename,
                                'map_subscribe_transient_local': 'false',
                                'set_initial_pose': 'true',
                                'x': str(pose['x_pose']),
                                'y': str(pose['y_pose']),
                                'yaw': str(pose['yaw']),
                                'local_costmap.local_costmap.ros__parameters.global_frame': robot_name+'/odom',
                                'global_costmap.global_costmap.ros__parameters.global_frame': 'map',
                                'recoveries_server.ros__parameters.global_frame': robot_name+'/odom',
                                'bt_navigator.ros__parameters.global_frame': 'map',
                                'robot_base_frame': robot_name+'/base_link'}

    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=robot_name,
            param_rewrites=param_substitutions,
            convert_types=True
        )


    urdf = os.path.join(bringup_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    start_robot_state_publiser_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'frame_prefix': robot_prefix,
            'use_sim_time': True,
            'robot_description': robot_description
            }],
        remappings=remappings
        )

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
                     'sim_2d': True
                     }])

    start_docking_client_cmd = Node(
        package="docking_controller",
        executable="docking_client",
        name="docking_client",
        namespace=namespace,
        parameters=[{'robot_id': robot_name,
                    'init_x_pose': pose['x_pose'],
                     'init_y_pose': pose['y_pose'],
                     'sim_2d': True}])
    ###########################################################################

    # Start nav2 pkg

    lifecycle_nodes = [ 'controller_server',
                        'planner_server',
                        'recoveries_server',
                        'bt_navigator',
                        'waypoint_follower']

    navigation_group_cmd = GroupAction([
        PushRosNamespace(
            namespace=namespace),

            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
                ),

            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
                ),

            Node(
                package='nav2_recoveries',
                executable='recoveries_server',
                name='recoveries_server',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
                ),

            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[configured_params],
                remappings=remappings
                ),

            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=[configured_params],
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

    # Add any actions
    ld.add_action(start_spawn_2d_cmd)
    
    # Add actions to launch naviagation nodes
    ld.add_action(start_robot_state_publiser_cmd)

    # Start nav2
    ld.add_action(navigation_group_cmd)

    # Robot functions
    ld.add_action(start_gazebo_battery_state_cmd)
    ld.add_action(start_detect_tag_pupil_cmd)
    ld.add_action(start_docking_controller_cmd)
    ld.add_action(start_docking_client_cmd)

    return ld
