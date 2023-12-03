from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="litterbug",
                executable="litterbug_service",
                name="litterbug_node",
                arguments=[
                    "--enable-vision-simulation",
                    "True",
                ],
            ),
        ]
    )
