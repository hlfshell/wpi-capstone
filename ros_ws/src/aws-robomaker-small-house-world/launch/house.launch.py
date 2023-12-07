import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# This launch file is used to launch the gazebo house world, yolo, tb3, nav2, slam_toolbox, and rviz2
def generate_launch_description():
    
    launch_file_dir = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )
<<<<<<<< HEAD:ros_ws/src/aws-robomaker-small-house-world/launch/house.launch.py

    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

========
    
    # House world packages
>>>>>>>> main:ros_ws/src/yolobot_gazebo/launch/house.yolo.launch.py
    house_world_path = os.path.join(
        get_package_share_directory("aws-robomaker-small-house-world"),
        "worlds",
        "small_house.world",
    )
    map_file = os.path.join(
        get_package_share_directory("aws-robomaker-small-house-world"),
        "maps",
        "house_1.yaml",
<<<<<<<< HEAD:ros_ws/src/aws-robomaker-small-house-world/launch/house.launch.py
    )

    params_file = os.path.join(get_package_share_directory("aws-robomaker-small-house-world"), 
                               "param", 
                               "tb3_nav_params.yaml"
========
>>>>>>>> main:ros_ws/src/yolobot_gazebo/launch/house.yolo.launch.py
    )

    config_dir = os.path.join(get_package_share_directory("explorer"), "config")
    rviz_config = os.path.join(config_dir, "nav.rviz")
<<<<<<<< HEAD:ros_ws/src/aws-robomaker-small-house-world/launch/house.launch.py
========
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    params_file = os.path.join(get_package_share_directory("aws-robomaker-small-house-world"), 
                               "param", 
                               "tb3_nav_params.yaml"
    )

    # Yolov8 packages
    pkg_yolobot_recognition = get_package_share_directory('yolobot_recognition')
>>>>>>>> main:ros_ws/src/yolobot_gazebo/launch/house.yolo.launch.py

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

<<<<<<<< HEAD:ros_ws/src/aws-robomaker-small-house-world/launch/house.launch.py
========
    # Yolov8 launch
    spawn_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_yolobot_recognition, 'launch', 'launch_yolov8.launch.py'),
        )
    )

    # Launch async slam toolbox
>>>>>>>> main:ros_ws/src/yolobot_gazebo/launch/house.yolo.launch.py
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory("nav2_bringup"),
                "/launch",
                "/bringup_launch.py",
            ]
        ),
        launch_arguments={"map": map_file, "params_file": params_file}.items(),
    )
    
    rviz = Node(
        package="rviz2",
        output="screen",
        executable="rviz2",
        name="rviz2_node",
        arguments=["-d", rviz_config],
    )

    explorer = Node(
        package="explorer",
        executable="explorer",
        name="explorer_node",
        output="screen",
        parameters=[],
    )

    nav_group = GroupAction(actions=[nav2, slam])

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(nav_group)
    #ld.add_action(explorer)
    ld.add_action(spawn_yolo)
    ld.add_action(rviz) 

    return ld