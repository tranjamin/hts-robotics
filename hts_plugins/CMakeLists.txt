cmake_minimum_required(VERSION 3.8)
project(hts_plugins)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_core REQUIRED)

add_library(hts_plugins
  src/hts_constraint_sampler.cpp
)

# target_compile_features(hts_plugins PUBLIC c_std_99 cxx_std_17)

target_include_directories(hts_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  hts_plugins
  pluginlib
  moveit_core
)

pluginlib_export_plugin_description_file(moveit_core
  plugins.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS hts_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  hts_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
