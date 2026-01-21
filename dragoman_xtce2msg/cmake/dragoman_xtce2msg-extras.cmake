# dragoman_xtce2msg-extras.cmake
# This file is automatically included by find_package(dragoman_xtce2msg)
# It makes the xtce_generate_messages() macro available to downstream packages

# Find the installed CMake module directory
if(dragoman_xtce2msg_DIR)
  # Include the xtce_generate_messages macro
  include("${dragoman_xtce2msg_DIR}/xtce_generate_messages.cmake")

  message(STATUS "dragoman_xtce2msg: xtce_generate_messages() macro is now available")
else()
  message(WARNING "dragoman_xtce2msg_DIR not set, cannot load xtce_generate_messages macro")
endif()
