from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    llm_service_node = Node(
        package="llm",
        executable="llm_service",
        name="llm_service_node",
        parameters=[{"provider": "openai"}],
    )

    return LaunchDescription([llm_service_node])
