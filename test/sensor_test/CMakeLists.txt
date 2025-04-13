project(sensor_test)

find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(GMock REQUIRED)
find_package(GTest REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
    ${GMOCK_INCLUDE_DIRS}
)

add_executable(map_by_time_test map_by_time_test.cpp)
target_link_libraries(map_by_time_test PUBLIC
  GTest::GTest
  GTest::Main
  common
  transform
  sensor
  ${GMOCK_LIBRARIES}
)

enable_testing()
add_test(NAME MapByTimeTest COMMAND map_by_time_test)