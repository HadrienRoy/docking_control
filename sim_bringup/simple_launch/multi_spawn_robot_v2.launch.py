import os
import math
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    
    # Get the launch directory
    bringup_dir = get_package_share_directory('sim_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    map_simulator_dir = get_package_share_directory('map_simulator')
    
    # Names and poses of the robots
    robots = [
        {'name': 'tb3_0', 'x_pose': -3.0, 'y_pose': 0.0, 'z_pose': 0.01, 
                          'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'tb3_1', 'x_pose': -3.0, 'y_pose': -3.0, 'z_pose': 0.01, 
                          'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'tb3_2', 'x_pose': -3.0, 'y_pose': 3.0, 'z_pose': 0.01, 
                          'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    ]

    # Launch configuration variables specific to simulation

    map_yaml_file = LaunchConfiguration('map')

    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    

    # Declare the launch arguments        
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'map.yaml'),
        description='Full path to map file to load')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view2.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Nav2 param files
    declare_waffle_pi_params_file_cmd = DeclareLaunchArgument(
        'waffle_pi_params_file',
        # default_value=os.path.join(bringup_dir, 'params', 'tb3_0_params.yaml'),
        default_value=os.path.join(bringup_dir, 'params', 'waffle_pi.yaml'),
        description='Full path to the ROS2 parameters file to use for waffle pi launched nodes')


    bt_xml_filename = os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Start simulation
    start_simulation2d_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(map_simulator_dir,'launch', 'simulation2d_launch.py')),
        launch_arguments={'map': map_yaml_file,
                          'map_server': 'true'}.items()
        )

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        # params_file = LaunchConfiguration(f"{robot['name']}_params_file")
        params_file = LaunchConfiguration("waffle_pi_params_file")

        param_substitutions = {
                                'use_sim_time': 'True',
                                'autostart': 'True',
                                'default_bt_xml_filename': bt_xml_filename,
                                'map_subscribe_transient_local': 'false',
                                'set_initial_pose': 'true',
                                'x': TextSubstitution(text=str(robot['x_pose'])),
                                'y': TextSubstitution(text=str(robot['y_pose'])),
                                'yaw': TextSubstitution(text=str(robot['yaw'])),
                                'local_costmap.local_costmap.ros__parameters.global_frame': robot['name']+'/odom',
                                'global_costmap.global_costmap.ros__parameters.global_frame': 'map',
                                'recoveries_server.ros__parameters.global_frame': robot['name']+'/odom',
                                'bt_navigator.ros__parameters.global_frame': 'map',
                                'robot_base_frame': robot['name']+'/base_link'}

        configured_params = RewrittenYaml(
            source_file=params_file,
            root_key=robot['name'],
            param_rewrites=param_substitutions,
            convert_types=True
        )

        group = GroupAction([
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #         os.path.join(launch_dir, 'rviz_launch.py')),
            #     condition=IfCondition(use_rviz),
            #     launch_arguments={'namespace': TextSubstitution(text=robot['name']),
            #                       'use_namespace': 'True',
            #                       'rviz_config': rviz_config_file}.items()),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                           'simple_launch',
                                                           'spawn_robot_v2.launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': map_yaml_file,
                                  'use_sim_time': 'true',
                                  'autostart': 'true',
                                  'use_rviz': 'False',
                                  'use_simulator': 'False',
                                  'headless': 'False',
                                  'use_robot_state_pub': use_robot_state_pub,
                                  'params_file': configured_params,
                                  'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                                  'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                                  'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                                  'roll': TextSubstitution(text=str(robot['roll'])),
                                  'pitch': TextSubstitution(text=str(robot['pitch'])),
                                  'yaw': TextSubstitution(text=str(robot['yaw'])),
                                  'robot_name':TextSubstitution(text=robot['name']), }.items()),
        ])

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    ld.add_action(declare_waffle_pi_params_file_cmd)

    # Add simulations
    ld.add_action(start_simulation2d_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld