import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ROBOT_ID = os.environ['ROBOT_ID']

    ld = LaunchDescription()

    detect_tag_pupil = Node(
        package="my_apriltag",
        executable="detect_tag_pupil",
        name="detect_tag_pupil"
    )

    docking_controller = Node(
        package="docking_controller",
        executable="docking_controller",
        name="docking_controller",
        parameters=[
            {"robot_id": ROBOT_ID},
        ]
    )

    docking_client = Node(
        package="docking_controller",
        executable="docking_client",
        name="docking_client"
        # parameters=[
        #     {"start_docking_controller": False},
        # ]
    )

    ld.add_action(detect_tag_pupil)
    ld.add_action(docking_controller)
    ld.add_action(docking_client)


    return ld
