from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="planner",
                executable="ai_service",
                name="ai_node",
                arguments=[],
            ),
        ]
    )
