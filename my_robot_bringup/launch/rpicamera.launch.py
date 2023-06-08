#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ROBOT_ID = os.environ['ROBOT_ID']

    camera_params = LaunchConfiguration(
        'camera_params',
        default=os.path.join(
            get_package_share_directory('my_robot_bringup'),
            'params',
            'camera.yaml'))

    return LaunchDescription([

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[camera_params],
            arguments=['--ros-args', '--remap', '__ns:=/'+ROBOT_ID+'/camera'],
            remappings=[
                ('/camera/image_raw', '/camera/image_raw/uncompressed'),
                
            ],
            output='screen',
            namespace=ROBOT_ID
            ),
    ])
