from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="query_services",
                executable="description_service",
                name="description_service_node",
            ),
            Node(
                package="query_services",
                executable="id_service",
                name="id_service_node",
            ),
            Node(
                package="query_services",
                executable="add_object_service",
                name="add_object_service_node",
            ),
            Node(
                package="query_services",
                executable="qa_service",
                name="qa_service_node",
            ),
            Node(
                package="query_services",
                executable="new_object",
                name="add_object_subscriber_node",
            ),
            Node(
                package="query_services",
                executable="rooms_service",
                name="rooms_service_node",
            ),
        ]
    )
