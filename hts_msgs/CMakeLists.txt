cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 17)

project(hts_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/RequestGrasp.srv"
)

ament_package()
