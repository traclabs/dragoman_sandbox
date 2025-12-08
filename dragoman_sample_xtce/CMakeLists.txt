cmake_minimum_required(VERSION 3.16.3)
project(dragoman_sample_xtce)

add_compile_options(-std=c++17)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(DEPS
  rclpy
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
  scripts/pymdb_generate_all_types.py
  scripts/pymdb_generate_curiosity_xtce.py
  scripts/pymdb_generate_imetro_demo_xtce.py
  scripts/pymdb_generate_multi_packet.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY xtce
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
