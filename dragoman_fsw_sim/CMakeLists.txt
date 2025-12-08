cmake_minimum_required(VERSION 3.16.3)
project(dragoman_fsw_sim)

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
  scripts/curiosity_simple_simulator.py
  scripts/imetro_demo_dummy_simulator.py
  scripts/imetro_demo_robot_simple_simulator.py
  scripts/all_types_simulator.py
  scripts/multipacket_simulator.py
  scripts/move_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
