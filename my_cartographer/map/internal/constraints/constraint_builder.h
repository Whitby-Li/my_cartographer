//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/map/proto/pose_graph/constraint_builder_options.pb.h"

namespace my_cartographer
{
  namespace map
  {
    namespace constraints
    {
      proto::ConstraintBuilderOptions CreateConstraintBuilderOptions(
          common::LuaParameterDictionary *parameter_dictionary);

    } // namespace constraints
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_CONSTRAINTS_CONSTRAINT_BUILDER_H