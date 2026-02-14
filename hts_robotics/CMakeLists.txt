cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(hts_robotics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(hts_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToPoint.action"
  "action/MoveTarget.action"
  "action/PickUpTarget.action"
  "action/GripperClose.action"
  "action/GripperOpen.action"
  DEPENDENCIES geometry_msgs
)


add_executable(hts_node src/hts_main.cpp)
ament_target_dependencies(hts_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  hts_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

target_link_libraries(hts_node
  ${cpp_typesupport_target}
)

install(TARGETS 
  hts_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install project launch files
install(
  DIRECTORY
    launch config objects
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
