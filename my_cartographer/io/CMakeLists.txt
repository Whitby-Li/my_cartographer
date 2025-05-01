project(io)

aux_source_directory(. IO_SRCS)
add_library(${PROJECT_NAME} SHARED ${IO_SRCS})
target_link_libraries(${PROJECT_NAME} 
  PUBLIC glog
  PRIVATE common)