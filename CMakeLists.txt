cmake_minimum_required(VERSION 3.8)
project(go2w_office_sim)

find_package(ament_cmake REQUIRED)

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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
