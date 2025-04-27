//
// Created by whitby on 2025-04-13.
//

#ifndef MY_CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_TEST_HELPERS_H_
#define MY_CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_TEST_HELPERS_H_

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/common/port.hpp"

#include <absl/memory/memory.h>
#include <glog/logging.h>

#include <memory>
#include <string>

using namespace my_cartographer::common;

namespace test
{
  class DummyFileResolver : public FileResolver
  {
  public:
    DummyFileResolver() {}

    DummyFileResolver(const DummyFileResolver &) = delete;
    DummyFileResolver &operator=(const DummyFileResolver &) = delete;

    ~DummyFileResolver() override {}

    std::string GetFileContentOrDie(const std::string &unused_basename) override
    {
      LOG(FATAL) << "Not implemented";
    }

    std::string GetFullPathOrDie(const std::string &unused_basename) override
    {
      LOG(FATAL) << "Not implemented";
    }
  };

  std::unique_ptr<LuaParameterDictionary> MakeDictionary(
      const std::string &code)
  {
    return absl::make_unique<LuaParameterDictionary>(
        code, absl::make_unique<DummyFileResolver>());
  }

} // namespace test

#endif // MY_CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_TEST_HELPERS_H_