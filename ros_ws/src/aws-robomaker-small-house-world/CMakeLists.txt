cmake_minimum_required(VERSION 3.5)
project(aws-robomaker-small-house-world)

find_package(gazebo_ros REQUIRED)
find_package(ament_cmake_ros REQUIRED)

install(DIRECTORY launch models worlds maps param
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(gazebo_ros)

ament_package()