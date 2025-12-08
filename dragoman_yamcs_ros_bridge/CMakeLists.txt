cmake_minimum_required(VERSION 3.16.3)
project(dragoman_yamcs_ros_bridge)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(DEPS
  rclpy
  sensor_msgs
  dragoman_msgs
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()

###########
## Build ##
###########

ament_export_dependencies(
  ${DEPS}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

#############
## Install ##
#############

# Install Python scripts
install(PROGRAMS
  scripts/yamcs_ros_bridge.py
  scripts/curiosity_yamcs_ros_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install configuration files
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
