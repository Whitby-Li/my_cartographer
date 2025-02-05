cmake_minimum_required(VERSION 3.16)

project(my_cartographer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(${PROJECT_SOURCE_DIR}/cmake/functions.cmake)
google_initialize_my_cartographer_project()
google_enable_testing()

set(MY_CARTOGRAPHER_SOURCE_DIR ${PROJECT_SOURCE_DIR})
set(MY_CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files
  CACHE PATH ".lua configuration files directory")

find_package(absl REQUIRED)
set(BOOST_COMPONENTS iostreams)
if(WIN32)
  list(APPEND BOOST_COMPONENTS zlib)
  set(Boost_USE_STATIC_LIBS FALSE)
endif()
find_package(Boost REQUIRED COMPONENTS ${BOOST_COMPONENTS})
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
)

add_subdirectory(common)
