project(io)

aux_source_directory(. IO_SRCS)
add_library(${PROJECT_NAME} SHARED ${IO_SRCS})
target_link_libraries(${PROJECT_NAME} 
  PUBLIC my_cartographer_dependencies
  PRIVATE common)

# 安装配置
install(TARGETS ${PROJECT_NAME}
  EXPORT my_cartographerTargets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)