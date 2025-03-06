cmake_minimum_required(VERSION 3.5)
project(oa_simulations)

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

add_executable(obstacle_avoidance
  src/main_swc.cpp
)

ament_target_dependencies(obstacle_avoidance
  rclcpp
  geometry_msgs
  sensor_msgs
)

target_link_libraries(obstacle_avoidance MainNode)

install(TARGETS
  obstacle_avoidance
  MainNode
  Solver
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional directories.
install(DIRECTORY
  launch
  rviz
  worlds
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