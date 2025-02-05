#ifndef MY_CARTOGRAPHER_COMMON_CONFIG_H_
#define MY_CARTOGRAPHER_COMMON_CONFIG_H_

namespace cartographer {
  namespace common {
  
  constexpr char kConfigurationFilesDirectory[] =
    "@MY_CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY@";
  constexpr char kSourceDirectory[] = "@MY_CARTOGRAPHER_SOURCE_DIR@";
  
  }  // namespace common
  }  // namespace cartographer
  
  #endif  // MY_CARTOGRAPHER_COMMON_CONFIG_H_