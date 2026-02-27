cmake_minimum_required(VERSION 3.16.3)
project(dragoman_fsw_sim)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


set(DEPS
  control_msgs
  rclcpp
  rclpy
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  tf2
  tf2_kdl
  trac_ik_lib
)

foreach(dependency IN ITEMS ${DEPS})
  find_package(${dependency} REQUIRED)
endforeach()


###########
## Build ##
###########
include_directories(include)

add_executable(arm_comm_udp_node
  src/arm_comm_udp_node.cpp
  src/arm_comm_udp.cpp
  src/serialize_arm_manual.cpp
)
ament_target_dependencies(arm_comm_udp_node ${DEPS} )

add_executable(mobile_servicing_system_comm_udp_node
  src/mobile_servicing_system_comm_udp_node.cpp
  src/mobile_servicing_system_comm_udp.cpp
  src/serialize_mobile_servicing_system_manual.cpp
)
ament_target_dependencies(mobile_servicing_system_comm_udp_node ${DEPS} )

add_executable(lunar_exploration_comm_udp_node
  src/lunar_exploration_comm_udp_node.cpp
  src/lunar_exploration_comm_udp.cpp
  src/serialize_lunar_exploration_manual.cpp
)
ament_target_dependencies(lunar_exploration_comm_udp_node ${DEPS} )


ament_export_dependencies(
  ${DEPS}
)

#############
## Install ##
#############

install(TARGETS
  arm_comm_udp_node
  mobile_servicing_system_comm_udp_node
  lunar_exploration_comm_udp_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/curiosity_simple_simulator.py
  scripts/imetro_demo_dummy_simulator.py
  scripts/imetro_demo_robot_simple_simulator.py
  scripts/gateway_demo_dummy_simulator.py
  scripts/all_types_simulator.py
  scripts/multipacket_simulator.py
  scripts/move_robot.py
  scripts/lunar_exploration_navigation_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
