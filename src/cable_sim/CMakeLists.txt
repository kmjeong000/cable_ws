cmake_minimum_required(VERSION 3.8)
project(cable_sim)

find_package(ament_cmake REQUIRED)

# Install headers only if you actually have them
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
  install(
    DIRECTORY include/
    DESTINATION include
  )
endif()

# Launch files
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
    FILES_MATCHING PATTERN "*.launch.py"
  )
endif()

# Models
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/models")
  install(
    DIRECTORY models/
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()

# Worlds
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/world")
  install(
    DIRECTORY world/
    DESTINATION share/${PROJECT_NAME}/world
  )
endif()


ament_package()
