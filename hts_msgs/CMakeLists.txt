cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)

project(hts_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestGrasp.srv"
  "srv/DisplayCloud.srv"
  "srv/EnableOrientationConstraints.srv"
  "action/MoveTarget.action"
  "action/PickUpTarget.action"
  "action/GripperClose.action"
  "action/GripperOpen.action"
  "action/GraspObject.action"
  DEPENDENCIES geometry_msgs
)

ament_package()
