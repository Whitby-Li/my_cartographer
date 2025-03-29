project(map)

# 生成 protobuf 文件
file(GLOB_RECURSE MAP_PROTOS "proto/*.proto")

foreach(ABS_FIL ${MAP_PROTOS})
  file(RELATIVE_PATH REL_FIL ${MY_CARTOGRAPHER_SOURCE_DIR} ${ABS_FIL})

  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND MAP_PROTOS_HDRS "${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
  list(APPEND MAP_PROTOS_SRCS "${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  message(STATUS "ABS_FIL is ${ABS_FIL}")
  message(STATUS "Generating ${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h and ${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")


  add_custom_command(
    OUTPUT "${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
            "${MY_CARTOGRAPHER_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${MY_CARTOGRAPHER_BINARY_DIR} -I
      ${MY_CARTOGRAPHER_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

aux_source_directory(. MAP_SRCS)
add_library(${PROJECT_NAME} SHARED ${MAP_SRCS} ${MAP_PROTOS_SRCS})
target_link_libraries(${PROJECT_NAME} 
  PUBLIC glog absl::base absl::strings
  PRIVATE common transform sensor
)