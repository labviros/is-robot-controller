cmake_minimum_required(VERSION 3.10.0)
include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(${CMAKE_BINARY_DIR}/conan_paths.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory("./src/is/robot-controller")
