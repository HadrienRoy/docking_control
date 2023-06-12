from simple_launch import SimpleLauncher
import yaml
from nav2_common.launch import RewrittenYaml, ReplaceString

def generate_launch_description():

    sl = SimpleLauncher()

    sl.declare_arg('name', default_value='turtlebot1')
    sl.declare_arg('sim_wheels', default_value=False)

    sl.declare_arg('map', default_value=sl.find('sim_bringup', 'map.yaml'))
    sl.declare_arg('map_server', default_value=True)

    sl.declare_arg('slam', default_value=False)
    sl.declare_arg('use_sim_time', default_value=True)
    sl.declare_arg('autostart', default_value=True)

    

    # force joints to 0 if wheels are not simulated
    zero_joints = sl.py_eval('not ', sl.arg('sim_wheels'))

    robots = [
        {'name': 'tb3_0', 'x_pose': -3.0, 'y_pose': 0.0, 'z_pose': 0.01, 
                          'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        {'name': 'tb3_1', 'x_pose': -3.0, 'y_pose': -3.0, 'z_pose': 0.01, 
                          'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
        # {'name': 'tb3_2', 'x_pose': -3.0, 'y_pose': 3.0, 'z_pose': 0.01, 
        #                   'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
    ]


    

    sl.include('map_simulator', 
               'simulation2d_launch.py', 
               launch_arguments= sl.arg_map(['map','map_server']))
    
    # run RViz2
    rviz_config_file = sl.find('sim_bringup', 'nav2_namespaced_view2.rviz')

    bt_xml_file = sl.find('nav2_bt_navigator', 'navigate_w_replanning_and_recovery.xml')
    
    for robot in robots:

        with sl.group(ns=robot['name']):

            # sl.node('rviz2','rviz2', 
            #          output='log',
            #          arguments=['-d', rviz_config_file,
            #             '--ros-args', '--log-level', 'error'])

            namespaced_rviz_config_file = ReplaceString(
                source_file=rviz_config_file,
                replacements={'<robot_namespace>': ('/', robot['name'])})

            # sl.node('rviz2',
            #         'rviz2', 
            #         #  namespace=robot['name'],
            #          output='screen',
            #          arguments=['-d', namespaced_rviz_config_file],
            #          remappings=[('/tf', 'tf'),
            #         ('/tf_static', 'tf_static'),
            #         ('/goal_pose', 'goal_pose'),
            #         ('/clicked_point', 'clicked_point'),
            #         ('/initialpose', 'initialpose')])
            
            # spawn robots with no localization

            sl.node('sim_bringup', 
                    'vel2joints.py',    
                    parameters = {'static_tf': True, 
                                  'x_pose': robot['x_pose'],
                                  'y_pose': robot['y_pose'],
                                  'yaw': robot['yaw']}
            )

            with sl.group(if_arg='sim_wheels'):
                sl.node('map_simulator', 'kinematics.py')

            # robot state publisher
            tf_prefix = sl.name_join(robot['name'], '/')
            sl.robot_state_publisher('sim_bringup', 
                                     'turtlebot3_waffle_pi.urdf.xacro', 
                                      xacro_args={'prefix': tf_prefix})

            # navigation nodes
            with open(sl.find('sim_bringup', 'nav2_nodes.yaml')) as f:
                nav2_nodes = yaml.safe_load(f)[sl.ros_version()]
                node_names = [executable for pkg, executable in nav2_nodes]

            # get default configuration file from nav2 (copied into lab4 package)
            nav2_params = sl.find('sim_bringup', f'nav2_params_{sl.ros_version()}.yaml')
            
            param_rewrites = {  'global_frame': robot['name']+'/odom',
                                'robot_base_frame': robot['name']+'/base_link',
                            #   'topic': robot['name']+'/scan',
                                'use_sim_time': sl.arg('use_sim_time'),
                                'autostart': sl.arg('autostart'),
                                'default_bt_xml_filename': bt_xml_file,
                                'map_subscribe_transient_local': 'True',
                            }

            configured_params = RewrittenYaml(
                source_file=nav2_params,
                root_key=robot['name'],
                param_rewrites=param_rewrites,
                convert_types=True)

            sl.declare_arg('params_file', default_value=configured_params)

            remappings = [('map', '/map'), ('/scan','scan'),
                            # ('/tf', 'tf'), ('/tf_static', 'tf_static')
                                ]

    
            # launch navigation nodes
            for pkg,executable in nav2_nodes:
                sl.node(pkg, executable, name=executable,
                    parameters=[configured_params],
                    remappings=remappings)

            # sl.node('nav2_bt_navigator', 'bt_navigator', name='bt_navigator',
            #     parameters=[configured_params],
            #     remappings=remappings)

            sl.node('nav2_lifecycle_manager','lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen', 
                    parameters={'autostart': True,
                                'node_names': node_names})


            # launch my nodes
            sl.node('sim_bringup',
                    'gazebo_battery_state',
                    parameters={'robot_id': robot['name']})

            sl.node('my_apriltag',
                    'detect_tag_pupil',
                    parameters={'robot_id': robot['name']})

            sl.node('docking_controller',
                    'docking_controller',
                    parameters={'robot_id': robot['name'],
                     'initial_approach_distance_tolerance': 0.4,
                     'final_approach_distance_tolerance': 0.2,
                     'init_x_pose': robot['x_pose'],
                     'init_y_pose': robot['y_pose'],})

            sl.node('docking_controller',
                    'docking_client',
                    parameters={'robot_id': robot['name']})

    return sl.launch_description()