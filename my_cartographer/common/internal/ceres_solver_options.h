//
// Created by whitby on 2025-02-04.
//

#ifndef MY_CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_
#define MY_CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/common/proto/ceres_solver_options.pb.h"

#include <ceres/ceres.h>

namespace my_cartographer
{
  namespace common
  {

    proto::CeresSolverOptions CreateCeresSolverOptionsProto(
        common::LuaParameterDictionary *parameter_dictionary);

    ceres::Solver::Options CreateCeresSolverOptions(
        const proto::CeresSolverOptions &proto);

  } // namespace common
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_
