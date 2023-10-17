from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription


def generate_launch_description():
    provider_argument = DeclareLaunchArgument(
        "provider", description="Which provider (openai or palm) to use"
    )

    llm_service_node = Node(
        package="llm", executable="llm_service", name="llm_service_node"
    )

    return LaunchDescription([provider_argument, llm_service_node])
