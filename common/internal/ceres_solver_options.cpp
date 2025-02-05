//
// Created by whitby on 2025-02-04.
//

#include "ceres_solver_options.h"

namespace cartographer
{
  namespace common
  {

    proto::CeresSolverOptions CreateCeresSolverOptionsProto(
        common::LuaParameterDictionary *parameter_dictionary)
    {
      proto::CeresSolverOptions proto;
      proto.set_use_nonmonotonic_steps(
          parameter_dictionary->GetBool("use_nonmonotonic_steps"));
      proto.set_max_num_iterations(
          parameter_dictionary->GetNonNegativeInt("max_num_iterations"));
      proto.set_num_threads(parameter_dictionary->GetNonNegativeInt("num_threads"));
      CHECK_GT(proto.max_num_iterations(), 0);
      CHECK_GT(proto.num_threads(), 0);
      return proto;
    }

    ceres::Solver::Options CreateCeresSolverOptions(
        const proto::CeresSolverOptions &proto)
    {
      ceres::Solver::Options options;
      options.use_nonmonotonic_steps = proto.use_nonmonotonic_steps();
      options.max_num_iterations = proto.max_num_iterations();
      options.num_threads = proto.num_threads();
      return options;
    }

  } // namespace common
} // namespace cartographer