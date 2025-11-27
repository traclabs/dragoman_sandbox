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

#include_directories(
# include
# ${orocos_kdl_INCLUDE_DIRS}
#)
#link_directories(${orocos_kdl_LIBRARY_DIRS}) # this is a hack, will eventually be unneeded once orocos-kdl is fixed

#add_library(craftsman_utils SHARED
#  src/frame_manager.cpp
#)
#ament_target_dependencies(craftsman_utils
#${DEPS}
#)
#target_link_libraries(craftsman_utils Eigen3::Eigen)


#ament_export_include_directories(include)
#ament_export_libraries(craftsman_utils interpolators srdf_publisher_node urdf_publisher_node)
ament_export_dependencies(
  ${DEPS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS
#    craftsman_utils
#  ARCHIVE DESTINATION lib
#  LIBRARY DESTINATION lib
#  RUNTIME DESTINATION lib/${PROJECT_NAME}
#)

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/move_robot.py
  scripts/pymdb_generate_imetro_demo_xtce.py
  scripts/imetro_demo_dummy_simulator.py
  scripts/imetro_demo_robot_simple_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY include/
#  DESTINATION include/
#)

install(
  DIRECTORY launch rviz urdf behaviors data xtce
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
