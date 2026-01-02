cmake_minimum_required(VERSION 3.16.3)
project(dragoman_xtce2msg)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python executable scripts
install(PROGRAMS
  scripts/xtce2msg
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install CMake modules for downstream packages
install(
  DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake
)

# Export dependencies needed by downstream packages
ament_export_dependencies(
  rclpy
  rosidl_default_generators
  builtin_interfaces
)

# Register the extras file so it's automatically included by find_package()
ament_package(
  CONFIG_EXTRAS cmake/dragoman_xtce2msg-extras.cmake
)
