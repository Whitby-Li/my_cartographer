//
// Created by whitby on 2025-05-05.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_OPTIONS_H_
#define MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_OPTIONS_H_

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/map/proto/pose_graph/optimization_problem_options.pb.h"

namespace my_cartographer
{
  namespace map
  {
    namespace optimization
    {

      proto::OptimizationProblemOptions CreateOptimizationProblemOptions(
          common::LuaParameterDictionary *parameter_dictionary);

    } // namespace optimization
  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_OPTIMIZATION_PROBLEM_OPTIONS_H_