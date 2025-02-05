//
// Created by whitby on 2025-02-04.
//

#ifndef MY_CARTOGRAPHER_COMMON_CONFIGURATION_FILE_RESOLVER_H_
#define MY_CARTOGRAPHER_COMMON_CONFIGURATION_FILE_RESOLVER_H_

#include "port.hpp"
#include "lua_parameter_dictionary.h"

#include <vector>

namespace cartographer
{
  namespace common
  {

    // A 'FileResolver' for the 'LuaParameterDictionary' that reads files from disk.
    // It searches the 'configuration_files_directories' in order to find the
    // requested filename. The last place searched is always the
    // 'configuration_files/' directory installed with Cartographer. It contains
    // reasonable configuration for the various Cartographer components which
    // provide a good starting ground for new platforms.
    class ConfigurationFileResolver : public FileResolver
    {
    public:
      explicit ConfigurationFileResolver(
          const std::vector<std::string> &configuration_files_directories);

      std::string GetFullPathOrDie(const std::string &basename) override;
      std::string GetFileContentOrDie(const std::string &basename) override;

    private:
      std::vector<std::string> configuration_files_directories_;
    };

  } // namespace common
} // namespace cartographer

#endif // MY_CARTOGRAPHER_COMMON_CONFIGURATION_FILE_RESOLVER_H_