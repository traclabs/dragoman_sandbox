cmake_minimum_required(VERSION 3.8)
project(dragoman_sample_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dragoman_xtce2msg REQUIRED)

# Generate messages from XTCE files using the macro
xtce_generate_messages(
  FILES
    xtce/AllTypes.xtce
    xtce/Curiosity.xtce
    xtce/Gateway.xtce
    xtce/IMetro.xtce
    xtce/MultiPacket.xtce
)

# Install XTCE files for use in YAMCS
install(DIRECTORY xtce
  DESTINATION share/${PROJECT_NAME})

ament_package()
