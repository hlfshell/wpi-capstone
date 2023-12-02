import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory("explorer"), "config")
    rviz_config = os.path.join(config_dir, "nav.rviz")

    ld = LaunchDescription()

    # Bring up our robot
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("turtlebot3_gazebo"),
                "/launch",
                # "/turtlebot3_house.launch.py",
                "/turtlebot3_world.launch.py",
            ]
        )
    )

    # Integrate our nav2 stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("nav2_bringup"),
                "/launch",
                "/navigation_launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    # Launch async slam toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("slam_toolbox"),
                "/launch",
                "/online_async_launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": "true",
            "publish_period": "0.0",
        }.items(),
    )

    # Rviz2 bringup
    rviz2 = Node(
        package="rviz2",
        output="screen",
        executable="rviz2",
        name="rviz2_node",
        arguments=["-d", rviz_config],
    )

    # Create a node for running our explorer application
    explorer = Node(
        package="explorer",
        executable="explorer",
        name="explorer_node",
        output="screen",
        parameters=[],
    )

    # Create a node for running our explorer application
    map_saver_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("nav2_map_server"),
                "/launch",
                "/map_saver_server.launch.py",
            ]
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    sim_group = GroupAction(actions=[gazebo])
    nav_group = GroupAction(actions=[nav2, slam, map_saver_server])
    ui_group = GroupAction(actions=[rviz2])
    capstone_group = GroupAction(actions=[explorer])

    ld.add_action(sim_group)
    ld.add_action(nav_group)
    ld.add_action(ui_group)

    ld.add_action(capstone_group)

    return ld
