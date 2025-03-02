project(transform_test)

find_package(Eigen3 REQUIRED)
find_package(LuaGoogle REQUIRED)
find_package(GMock REQUIRED)
find_package(GTest REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${LUA_INCLUDE_DIR}
    ${GMOCK_INCLUDE_DIRS}
)

add_executable(rigid_transform_test rigid_transform_test.cpp)
target_link_libraries(rigid_transform_test PUBLIC
  GTest::GTest
  GTest::Main
  common
  transform
  ${GMOCK_LIBRARIES}
)


enable_testing()
add_test(NAME RigidTransformTest COMMAND rigid_transform_test)
