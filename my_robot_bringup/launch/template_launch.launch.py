# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
    
#     ld = LaunchDescription()

#     remap_topic = ("topic_name", "remap_topic_name")

#     node_name = Node(
#         package="pkg_name",
#         executable="executable_name",
#         name="node_name",
#         remappings=[
#             remap_topic
#         ],
#         parameters=[
#             {"param1": val},
#             {"param2": val}
#         ]
#     )

#     ld.add_action(node_name)

#     return ld