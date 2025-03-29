find_package(absl REQUIRED)
find_package(Protobuf 3.0.0 REQUIRED)
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
add_subdirectory(transform)
add_subdirectory(metrics)
add_subdirectory(sensor)
add_subdirectory(map)