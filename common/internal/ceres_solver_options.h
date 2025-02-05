//
// Created by whitby on 2025-02-04.
//

#ifndef MY_CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_
#define MY_CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_

#include "lua_parameter_dictionary.h"
#include "ceres_solver_options.pd.h"

#include <ceres/ceres.h>

namespace cartographer
{
  namespace common
  {

    proto::CeresSolverOptions CreateCeresSolverOptionsProto(
        common::LuaParameterDictionary *parameter_dictionary);

    ceres::Solver::Options CreateCeresSolverOptions(
        const proto::CeresSolverOptions &proto);

  } // namespace common
} // namespace cartographer

#endif // MY_CARTOGRAPHER_COMMON_CERES_SOLVER_OPTIONS_H_
