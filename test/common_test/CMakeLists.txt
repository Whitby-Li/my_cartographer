project(common_test)

find_package(GTest REQUIRED)

include_directories(
  ${MY_CARTOGRAPHER_SOURCE_DIR}
)

add_executable(thread_pool_test thread_pool_test.cpp)
target_link_libraries(thread_pool_test PUBLIC
  GTest::GTest
  GTest::Main
  absl::base
  common)

enable_testing()
add_test(NAME ThreadPoolTest COMMAND thread_pool_test)
