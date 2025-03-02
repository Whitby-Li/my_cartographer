project(metrics)

aux_source_directory(. METRICS_SRCS)
add_library(${PROJECT_NAME} SHARED ${METRICS_SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC 
  glog 
)