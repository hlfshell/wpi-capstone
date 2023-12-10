import launch

from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
import launch_ros
import os
import xacro

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='fetch_description').find('fetch_description')
    default_model_path = os.path.join(pkg_share, 'robots/fetch.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    params_file = os.path.join(pkg_share, "config","nav2_params.yaml")
    
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

    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    doc = xacro.parse(open(default_model_path))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': True}]
    )

    house_spawner = Node(
        package="autonomous_tb3",
        output="screen",
        executable="sdf_spawner",
        name="maze_spawner",
       arguments=[house_world_path, "b", "0.0", "0.0"],
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


    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'fetch', '-topic', 'robot_description', '-x', x_pose, '-y', y_pose],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        gzserver_cmd,
        gzclient_cmd,
        robot_state_publisher,
        house_spawner,
        nav2,
        #slam,
        spawn_entity,
        robot_localization_node,
        rviz_node

    ])