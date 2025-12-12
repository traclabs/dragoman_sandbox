cmake_minimum_required(VERSION 3.8)
project(dragoman_generated_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Automatically find all .msg files in the msg directory
file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")

if(EXISTS ${MSG_FILES})
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
    DEPENDENCIES builtin_interfaces
  )
endif()


ament_package()
