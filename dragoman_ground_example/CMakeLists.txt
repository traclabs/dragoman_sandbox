cmake_minimum_required(VERSION 3.16.3)
project(dragoman_ground_example)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/imetro_joint_state_publisher.py
    scripts/curiosity_joint_state_publisher.py
    scripts/gateway_joint_state_publisher.py
    scripts/mobile_servicing_system_joint_state_publisher.py
    scripts/lunar_exploration_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
