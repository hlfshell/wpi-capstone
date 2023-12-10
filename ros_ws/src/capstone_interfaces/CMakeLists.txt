cmake_minimum_required(VERSION 3.8)
project(capstone_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/LLM.srv"
  "srv/ObjectDescriptionQuery.srv"
  "srv/ObjectIDQuery.srv"
  "srv/PlannerQuery.srv"
  "srv/AddObject.srv"
  "msg/ObjectSpotted.msg"
  "msg/StateObject.msg"
  "srv/PickUpObject.srv"
  "srv/GiveObject.srv"
  "msg/Room.msg"
  "srv/GetRooms.srv"
  "srv/RoomByCoordinates.srv"
  "msg/HumanSpotted.msg"
  "msg/HumanPose.msg"
  "srv/QueryHumanLocation.srv"
  "msg/Plan.msg"
  "msg/Objective.msg"
  "msg/ObjectiveStatus.msg"
  "msg/AIPrintStatement.msg"
  "msg/RobotAction.msg"
  "msg/Item.msg"
  "msg/ItemsState.msg"
  DEPENDENCIES builtin_interfaces
)

ament_package()
