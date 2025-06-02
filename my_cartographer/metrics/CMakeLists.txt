project(metrics)

aux_source_directory(. METRICS_SRCS)
add_library(${PROJECT_NAME} SHARED ${METRICS_SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC 
  my_cartographer_dependencies 
)

# 安装配置
install(TARGETS ${PROJECT_NAME}
  EXPORT my_cartographerTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)