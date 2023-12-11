from launch import LaunchDescription, get_package_share_directory
from launch_ros.actions import Node

PACKAGE_NAME = "robot"
PACKAGE_DIR = get_package_share_directory(PACKAGE_NAME)

URDF_FILE = PACKAGE_DIR + "/urdf/robot.urdf"

def generate_launch_description():
    spawn_fetch_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        parameters=[{"-entity": "robot", "-file", URDF_FILE}],
    )

    return LaunchDescription([spawn_fetch_node])
