cmake_minimum_required(VERSION 3.16.3)
project(dragoman_sandbox)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


set(DEPS
  rclcpp
  rclpy
  geometry_msgs
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
  scripts/move_robot.py
  scripts/pymdb_generate_imetro_demo_xtce.py
  scripts/pymdb_generate_curiosity_xtce.py
  scripts/imetro_demo_dummy_simulator.py
  scripts/imetro_demo_robot_simple_simulator.py
  scripts/curiosity_simple_simulator.py
  scripts/imetro_yamcs
  scripts/curiosity_yamcs
  scripts/curiosity_yamcs_ros_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz urdf behaviors data xtce config
  DESTINATION share/${PROJECT_NAME}
)

# Install the entire imetro_yamcs_project directory structure so it can run from install
install(
  DIRECTORY imetro_yamcs_project/
  DESTINATION share/${PROJECT_NAME}/imetro_yamcs_project
  USE_SOURCE_PERMISSIONS
  PATTERN "target" EXCLUDE
)

# Install the entire curiosity_yamcs_project directory structure so it can run from install
install(
  DIRECTORY curiosity_yamcs_project/
  DESTINATION share/${PROJECT_NAME}/curiosity_yamcs_project
  USE_SOURCE_PERMISSIONS
  PATTERN "target" EXCLUDE
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
