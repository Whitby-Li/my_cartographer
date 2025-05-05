//
// Created by whitby on 2025-05-03.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_SPA_COST_FUNCTION_2D_H_
#define MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_SPA_COST_FUNCTION_2D_H_

#include "my_cartographer/map/pose_graph_interface.h"

#include "ceres/ceres.h"

namespace my_cartographer
{
  namespace map
  {
    namespace optimization
    {

      ceres::CostFunction *CreateAutoDiffSpaCostFunction(
          const PoseGraphInterface::Constraint::Pose &pose);

      ceres::CostFunction *CreateAnalyticalSpaCostFunction(
          const PoseGraphInterface::Constraint::Pose &pose);

    } // namespace optimization
  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_SPA_COST_FUNCTION_2D_H_