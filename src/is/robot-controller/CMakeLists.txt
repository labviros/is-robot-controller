cmake_minimum_required(VERSION 3.10.0)

find_package(is-wire REQUIRED)
find_package(is-msgs REQUIRED)
find_package(boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Protobuf REQUIRED)
get_target_property(Protobuf_IMPORT_DIRS is-msgs::is-msgs INTERFACE_INCLUDE_DIRECTORIES)
set(PROTOBUF_GENERATE_CPP_APPEND_PATH OFF)
PROTOBUF_GENERATE_CPP(options_src options_hdr "conf/options.proto")

add_executable(service.bin
  "service.cpp"
  ${options_src}
  ${options_hdr}
  "extended-kalman-filter.hpp"
  "extended-kalman-filter.cpp"
  "pose-estimation.hpp"
  "pose-estimation.cpp"
  "inverse-kinematics-controller.hpp"
  "inverse-kinematics-controller.cpp"
  "subscription-manager.hpp"
  "subscription-manager.cpp"
  "basic-move-task.hpp"
  "basic-move-task.cpp"
  "control-task.hpp"
)

target_link_libraries(
  service.bin
 PUBLIC
  is-msgs::is-msgs
  is-wire::is-wire
  boost::boost
  Eigen3::Eigen
)

target_include_directories(
  service.bin
 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}> # for headers when building
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # for generated files in build mode
)

set_property(TARGET service.bin PROPERTY CXX_STANDARD 14)
