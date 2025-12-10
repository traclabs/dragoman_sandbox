cmake_minimum_required(VERSION 3.16.3)
project(dragoman_yamcs_project)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


set(DEPS
  rclcpp
  rclpy
  geometry_msgs
  dragoman_generated_msgs
  sensor_msgs
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

#############
## Install ##
#############

install(PROGRAMS
  scripts/dragoman_yamcs
  DESTINATION lib/${PROJECT_NAME}
)

# Install the entire dragoman_yamcs_project directory structure so it can run from install
install(
  DIRECTORY src
  DESTINATION share/${PROJECT_NAME}/dragoman_yamcs_project
  USE_SOURCE_PERMISSIONS
  PATTERN "target" EXCLUDE
)

install(
  FILES pom.xml
  DESTINATION share/${PROJECT_NAME}/dragoman_yamcs_project
)

ament_package()
