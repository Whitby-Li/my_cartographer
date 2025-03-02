project(common)

configure_file(
  ${PROJECT_SOURCE_DIR}/config.h.cmake
  ${PROJECT_BINARY_DIR}/config.h
)

file(GLOB_RECURSE COMMON_PROTOS "proto/*.proto")

foreach(ABS_FIL ${COMMON_PROTOS})
  file(RELATIVE_PATH REL_FIL ${MY_CARTOGRAPHER_SOURCE_DIR} ${ABS_FIL})

  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND COMMON_PROTO_HDRS "${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
  list(APPEND COMMON_PROTO_SRCS "${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")

  message(STATUS "Generating ${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h and ${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  message(STATUS "PROTOBUF_PROTOC_EXECUTABLE is ${PROTOBUF_PROTOC_EXECUTABLE}")
  message(STATUS "ABS_FIL is ${ABS_FIL}")

  add_custom_command(
    OUTPUT "${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
            "${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out ${MY_CARTOGRAPHER_BINARY_DIR} -I
      ${MY_CARTOGRAPHER_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

aux_source_directory(. COMMON_SRCS)
message(STATUS "common sources: ${COMMON_SRCS}")
add_library(${PROJECT_NAME} SHARED ${COMMON_SRCS} ${COMMON_PROTO_SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC 
  glog 
  absl::base
  absl::memory
  absl::str_format
  absl::strings
  absl::hash
  absl::raw_hash_set
  ${LUA_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${CERES_LIBRARIES} 
  ${Boost_LIBRARIES})