cmake_minimum_required(VERSION 3.10.0)
include(GNUInstallDirs)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include(${CMAKE_BINARY_DIR}/conan_paths.cmake)

add_subdirectory("./src/is/robot-controller")
