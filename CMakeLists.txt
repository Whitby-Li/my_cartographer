cmake_minimum_required(VERSION 3.16)

project(my_cartographer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${PROJECT_SOURCE_DIR}/cmake/functions.cmake)
google_initialize_my_cartographer_project()
google_enable_testing()

set(MY_CARTOGRAPHER_SOURCE_DIR ${PROJECT_SOURCE_DIR})
set(MY_CARTOGRAPHER_BINARY_DIR ${PROJECT_BINARY_DIR})
set(MY_CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files
  CACHE PATH ".lua configuration files directory")

include_directories(
  ${MY_CARTOGRAPHER_SOURCE_DIR}
  ${MY_CARTOGRAPHER_BINARY_DIR}
)

add_subdirectory(my_cartographer)
add_subdirectory(test)
