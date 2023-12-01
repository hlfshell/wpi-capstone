from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory("autonomous_tb3"), "config")

    map_file = os.path.join(config_dir, "tb3_world.yaml")
    params_file = os.path.join(config_dir, "tb3_nav_params.yaml")
    rviz_config = os.path.join(config_dir, "tb3_nav.rviz")

    return LaunchDescription([
        # Bring up our robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('turtlebot3_gazebo'), '/launch', '/turtlebot3_world.launch.py']
            )
        ),

        # Integrate our nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py']
            ),
            launch_arguments={
                "map": map_file,
                "params_file": params_file,
                "use_sim_time": "true",
            }.items(),
        ),

        # Rviz2 bringup
        Node(
            package="rviz2",
            output="screen",
            executable="rviz2",
            name="rviz2_node",
            arguments=[
                "-d", rviz_config
            ],
        )
    ])