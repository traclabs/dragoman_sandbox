cmake_minimum_required(VERSION 3.16.3)
project(dragoman_yamcs_ros_bridge)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/yamcs_ros_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
