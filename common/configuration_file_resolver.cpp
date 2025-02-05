//
// Created by whitby on 2025-02-04.
//

#include "configuration_file_resolver.h"
#include "config.h"

#include <glog/logging.h>

#include <fstream>
#include <iostream>
#include <streambuf>

namespace cartographer
{
  namespace common
  {

    ConfigurationFileResolver::ConfigurationFileResolver(
        const std::vector<std::string> &configuration_files_directories)
        : configuration_files_directories_(configuration_files_directories)
    {
      configuration_files_directories_.push_back(kConfigurationFilesDirectory);
    }

    std::string ConfigurationFileResolver::GetFullPathOrDie(
        const std::string &basename)
    {
      for (const auto &path : configuration_files_directories_)
      {
        const std::string filename = path + "/" + basename;
        std::ifstream stream(filename.c_str());
        if (stream.good())
        {
          LOG(INFO) << "Found '" << filename << "' for '" << basename << "'.";
          return filename;
        }
      }
      LOG(FATAL) << "File '" << basename << "' was not found.";
    }

    std::string ConfigurationFileResolver::GetFileContentOrDie(
        const std::string &basename)
    {
      CHECK(!basename.empty()) << "File basename cannot be empty." << basename;
      const std::string filename = GetFullPathOrDie(basename);
      std::ifstream stream(filename.c_str());
      return std::string((std::istreambuf_iterator<char>(stream)),
                         std::istreambuf_iterator<char>());
    }

  } // namespace common
} // namespace cartographer