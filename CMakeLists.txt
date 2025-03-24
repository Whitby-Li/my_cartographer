cmake_minimum_required(VERSION 3.16)

project(my_cartographer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${PROJECT_SOURCE_DIR}/cmake/functions.cmake)
google_initialize_my_cartographer_project()
google_enable_testing()

set(MY_CARTOGRAPHER_SOURCE_DIR ${PROJECT_SOURCE_DIR})
set(MY_CARTOGRAPHER_BINARY_DIR ${PROJECT_BINARY_DIR})
set(MY_CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files
  CACHE PATH ".lua configuration files directory")

# 基础 C++ 依赖
find_package(Eigen3 REQUIRED)
find_package(absl REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)

include_directories(
  ${MY_CARTOGRAPHER_SOURCE_DIR}
  ${MY_CARTOGRAPHER_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${PROTOBUF_INCLUDE_DIRS}
)

add_subdirectory(my_cartographer)
add_subdirectory(test)
