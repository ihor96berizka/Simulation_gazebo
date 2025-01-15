cmake_minimum_required(VERSION 3.5)
project(assignment_3a)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Besides the package specific dependencies we also need the `pluginlib` and `webots_ros2_driver`
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(solver)
add_subdirectory(main_node)

add_executable(main_swc
  src/main_swc.cpp
)

ament_target_dependencies(main_swc
  rclcpp
  geometry_msgs
  sensor_msgs
)

target_link_libraries(main_swc MainNode)

install(TARGETS
  main_swc
  MainNode
  Solver
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional directories.
install(DIRECTORY
  launch
  resource
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_package()