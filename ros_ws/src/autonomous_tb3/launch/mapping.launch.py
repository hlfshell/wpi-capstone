from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    config_dir = os.path.join(get_package_share_directory("autonomous_tb3"), "config")

    return LaunchDescription([
        Node(
            package='cartographer_ros',
            output='screen',
            executable='cartographer_node',
            name='cr_node',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', "turtlebot3_mapping.lua",
            ]
        ),
        Node(
            package='cartographer_ros',
            output='screen',
            executable='cartographer_occupancy_grid_node',
            name='cr_oc_node'
        ),
    ])