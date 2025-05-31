cmake_minimum_required(VERSION 3.16)

project(my_cartographer)
set(PROJECT_VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(${PROJECT_SOURCE_DIR}/cmake/functions.cmake)
google_initialize_my_cartographer_project()
google_enable_testing()

set(MY_CARTOGRAPHER_SOURCE_DIR ${PROJECT_SOURCE_DIR})
set(MY_CARTOGRAPHER_BINARY_DIR ${PROJECT_BINARY_DIR})
set(MY_CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files
  CACHE PATH ".lua configuration files directory")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 基础 C++ 依赖
find_package(Eigen3 REQUIRED)
find_package(absl REQUIRED CONFIG)
find_package(Protobuf 3.0.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(LuaGoogle REQUIRED)

include_directories(
  ${MY_CARTOGRAPHER_SOURCE_DIR}
  ${MY_CARTOGRAPHER_BINARY_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${LUA_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIRS}
)

add_subdirectory(my_cartographer/common)
add_subdirectory(my_cartographer/transform)
add_subdirectory(my_cartographer/metrics)
add_subdirectory(my_cartographer/sensor)
add_subdirectory(my_cartographer/io)
add_subdirectory(my_cartographer/map)
# add_subdirectory(test)

# 安装头文件
install(
  DIRECTORY my_cartographer/
  DESTINATION include/my_cartographer
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# 所有子模块导出 target 注册 my_cartographer 命名空间
install(EXPORT my_cartographerTargets
  FILE my_cartographerTargets.cmake
  NAMESPACE my_cartographer::
  DESTINATION lib/cmake/my_cartographer
)

set(PACKAGE_INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/include")

# 生成配置文件
include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/my_cartographerConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/my_cartographerConfig.cmake"
  INSTALL_DESTINATION lib/cmake/my_cartographer
  PATH_VARS PACKAGE_INCLUDE_INSTALL_DIR
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/my_cartographerConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/my_cartographerConfigVersion.cmake"
  DESTINATION lib/cmake/my_cartographer
)