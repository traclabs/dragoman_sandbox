# xtce_generate_messages.cmake
# CMake macro to generate ROS 2 messages from XTCE files
#
# This macro automates the pipeline: XTCE → .msg → compiled C++/Python interfaces
#
# Usage:
#   find_package(dragoman_xtce2msg REQUIRED)
#
#   xtce_generate_messages(
#     FILES
#       xtce/Gateway.xtce
#       xtce/IMetro.xtce
#   )
#

macro(xtce_generate_messages)
  # Parse arguments
  cmake_parse_arguments(
    ARG
    ""
    ""
    "FILES"
    ${ARGN}
  )

  # Validate that FILES were provided
  if(NOT ARG_FILES)
    message(FATAL_ERROR "xtce_generate_messages: No XTCE files specified. Use FILES argument.")
  endif()

  # Find Python interpreter
  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  # Ensure rosidl_default_generators is available
  find_package(rosidl_default_generators REQUIRED)

  # Create output directory and generate .msg files from XTCE at configure time
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/msg")
  message(STATUS "xtce_generate_messages: Converting XTCE files to .msg format...")

  foreach(XTCE_FILE ${ARG_FILES})
    # Get absolute path
    if(NOT IS_ABSOLUTE ${XTCE_FILE})
      set(XTCE_FILE_ABS "${CMAKE_CURRENT_SOURCE_DIR}/${XTCE_FILE}")
    else()
      set(XTCE_FILE_ABS "${XTCE_FILE}")
    endif()

    # Validate XTCE file exists
    if(NOT EXISTS ${XTCE_FILE_ABS})
      message(FATAL_ERROR
        "xtce_generate_messages: XTCE file not found: ${XTCE_FILE_ABS}\n"
        "Make sure the file exists before building."
      )
    endif()

    # Run xtce2msg to generate .msg files
    execute_process(
      COMMAND ${Python3_EXECUTABLE} -m dragoman_xtce2msg.xtce2msg
              ${XTCE_FILE_ABS} "${CMAKE_CURRENT_BINARY_DIR}/msg"
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      RESULT_VARIABLE XTCE_RESULT
      OUTPUT_VARIABLE XTCE_OUTPUT
      ERROR_VARIABLE XTCE_ERROR
    )

    if(NOT XTCE_RESULT EQUAL 0)
      message(FATAL_ERROR
        "xtce_generate_messages: Failed to convert ${XTCE_FILE}\n"
        "Output: ${XTCE_OUTPUT}\n"
        "Error: ${XTCE_ERROR}"
      )
    endif()

    message(STATUS "  Converted: ${XTCE_FILE}")
  endforeach()

  # Find and format generated .msg files for rosidl_generate_interfaces
  file(GLOB_RECURSE GENERATED_MSG_FILES_ABS "${CMAKE_CURRENT_BINARY_DIR}/msg/*.msg")

  if(NOT GENERATED_MSG_FILES_ABS)
    message(FATAL_ERROR "xtce_generate_messages: No .msg files were generated")
  endif()

  # Format as "absolute_base_path:relative/path/to/file.msg"
  set(GENERATED_MSG_FILES "")
  foreach(MSG_FILE_ABS ${GENERATED_MSG_FILES_ABS})
    file(RELATIVE_PATH MSG_FILE_REL ${CMAKE_CURRENT_BINARY_DIR} ${MSG_FILE_ABS})
    list(APPEND GENERATED_MSG_FILES "${CMAKE_CURRENT_BINARY_DIR}:${MSG_FILE_REL}")
  endforeach()

  list(LENGTH GENERATED_MSG_FILES_ABS MSG_COUNT)
  message(STATUS "xtce_generate_messages: Generated ${MSG_COUNT} .msg file(s)")

  # Call rosidl_generate_interfaces with generated .msg files
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${GENERATED_MSG_FILES}
    DEPENDENCIES builtin_interfaces
  )

  message(STATUS "xtce_generate_messages: Configuration complete for ${PROJECT_NAME}")

endmacro()
