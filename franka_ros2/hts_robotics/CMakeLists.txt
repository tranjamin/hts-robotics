cmake_minimum_required(VERSION 3.5)

# Suppress CMP0002 warning 
option(CHECK_TIDY "Enable clang-tidy checks" OFF)
option(CMAKE_EXPORT_COMPILE_COMMANDS "Enable compile commands export" OFF)


project(hts_robotics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hts_node src/hts_main.cpp)
target_link_libraries(hts_node PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})

install(TARGETS 
  hts_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install project launch files
install(
  DIRECTORY
    launch config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
