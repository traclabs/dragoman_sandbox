cmake_minimum_required(VERSION 3.16.3)
project(dragoman_xtce2msg)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

###########
## Build ##
###########

ament_export_dependencies(
  rclpy
)

#############
## Install ##
#############

# Install Python executable scripts
install(PROGRAMS
  scripts/xtce2msg
  scripts/generate_msgs.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
