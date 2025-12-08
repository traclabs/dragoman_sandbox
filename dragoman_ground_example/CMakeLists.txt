cmake_minimum_required(VERSION 3.16.3)
project(dragoman_ground_example)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
