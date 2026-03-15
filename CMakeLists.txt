cmake_minimum_required(VERSION 3.8)
project(go2w_office_sim)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(frontier_explorer src/frontier_explorer.cpp)
target_compile_features(frontier_explorer PUBLIC cxx_std_17)
ament_target_dependencies(
  frontier_explorer
  geometry_msgs
  nav2_msgs
  nav_msgs
  rclcpp
  rclcpp_action
  std_msgs
  tf2_msgs
  visualization_msgs
)

install(DIRECTORY
  launch
  urdf
  meshes
  worlds
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/frontier_explorer.py
  scripts/monitor_ros_node_cpu.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  frontier_explorer
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
