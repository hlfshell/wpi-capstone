import os
#import launch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    house_world_path = os.path.join(
        get_package_share_directory("aws-robomaker-small-house-world"),
        "worlds",
        "small_house.world",
    )
    map_file = os.path.join(
        get_package_share_directory("aws-robomaker-small-house-world"),
        "maps",
        "map.yaml",
    )

    config_dir = os.path.join(get_package_share_directory("autonomous_tb3"), "config")
    params_file = os.path.join(config_dir, "tb3_nav_params.yaml")
    rviz_config = os.path.join(config_dir, "tb3_nav.rviz")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": house_world_path}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    house_spawner = Node(
        package="autonomous_tb3",
        output="screen",
        executable="sdf_spawner",
        name="maze_spawner",
       arguments=[house_world_path, "b", "0.0", "0.0"],
    )
    
    small_house = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('aws-robomaker-small-house-world'),
                'launch',
                'view_small_house.launch.py'
            )
        )
    )

    # Include this if you're mapping the maze via keyboard
    maze_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
    )

    # # Remove this if you're mapping the maze via keyboard
    # maze_nav = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             get_package_share_directory("nav2_bringup"),
    #             "/launch",
    #             "/bringup_launch.py",
    #         ]
    #     ),
    #     launch_arguments={"map": map_file, "params_file": params_file}.items(),
    # )
    
    rviz = Node(
        package="rviz2",
        output="screen",
        executable="rviz2",
        name="rviz2_node",
        arguments=["-d", rviz_config],
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(house_spawner)
    #ld.add_action(small_house)
    ld.add_action(maze_mapping)
    ld.add_action(rviz)
    # ld.add_action(maze_nav)
    

    return ld