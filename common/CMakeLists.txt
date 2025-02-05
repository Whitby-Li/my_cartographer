project(common)

configure_file(
  ${PROJECT_SOURCE_DIR}/config.h.cmake
  ${PROJECT_BINARY_DIR}/config.h
)

file(GLOB_RECURSE COMMON_PROTOS "proto/*.proto")
foreach(ABS_FIL ${COMMON_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND COMMON_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

aux_source_directory(. COMMON_SRCS)
message(STATUS "common sources: ${COMMON_SRCS}")
add_library(${PROJECT_NAME} STATIC ${COMMON_SRCS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_BINARY_DIR})