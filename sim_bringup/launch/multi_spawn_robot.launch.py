import os
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
    
    # Names and poses of the robots
    robots = [
        {'name': 'tb3_0', 'x_pose': -3.0, 'y_pose': 0.0, 'z_pose': 0.01, 
                          'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'tb3_1', 'x_pose': -3.0, 'y_pose': -3.0, 'z_pose': 0.01, 
                          'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        # {'name': 'tb3_2', 'x_pose': -3.0, 'y_pose': 3.0, 'z_pose': 0.01, 
        #                   'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    ]

    # Launch configuration variables specific to simulation
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    map_yaml_file = LaunchConfiguration('map')

    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(bringup_dir, 'worlds', 'wall_world.model'),
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')
        
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'map.yaml'),
        description='Full path to map file to load')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
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
    declare_tb3_0_params_file_cmd = DeclareLaunchArgument(
        'tb3_0_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'tb3_0_params.yaml'),
        description='Full path to the ROS2 parameters file to use for tb3_0 launched nodes')

    declare_tb3_1_params_file_cmd = DeclareLaunchArgument(
        'tb3_1_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'tb3_1_params.yaml'),
        description='Full path to the ROS2 parameters file to use for tb3_1 launched nodes')

    declare_tb3_2_params_file_cmd = DeclareLaunchArgument(
        'tb3_2_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'tb3_2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for tb3_2 launched nodes')

    declare_tb3_3_params_file_cmd = DeclareLaunchArgument(
        'tb3_3_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'tb3_3_params.yaml'),
        description='Full path to the ROS2 parameters file to use for tb3_3 launched nodes')


    # Start Gazebo with plugin providing the robot spawning service
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_init.so',
                                     '-s', 'libgazebo_ros_factory.so', world],
        output='screen')
    
    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(f"{robot['name']}_params_file")

        param_substitutions = {'set_initial_pose': 'true',
                               'x': TextSubstitution(text=str(robot['x_pose'])),
                               'y': TextSubstitution(text=str(robot['y_pose'])),
                               'yaw': TextSubstitution(text=str(robot['yaw']))}

        configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True
        )

        group = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'rviz_launch.py')),
                condition=IfCondition(use_rviz),
                launch_arguments={'namespace': TextSubstitution(text=robot['name']),
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file}.items()),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                           'launch',
                                                           'spawn_robot.launch.py')),
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
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    ld.add_action(declare_tb3_0_params_file_cmd)
    ld.add_action(declare_tb3_1_params_file_cmd)
    ld.add_action(declare_tb3_2_params_file_cmd)
    ld.add_action(declare_tb3_3_params_file_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld