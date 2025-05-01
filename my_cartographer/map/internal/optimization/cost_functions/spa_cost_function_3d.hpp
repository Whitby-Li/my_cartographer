//
// Created by whitby on 2025-04-26.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_HPP_
#define MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_HPP_

#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <ceres/jet.h>

#include "my_cartographer/common/math.hpp"
#include "my_cartographer/map/internal/optimization/cost_functions/cost_helpers.hpp"
#include "my_cartographer/map/pose_graph.h"
#include "my_cartographer/transform/transform.h"

namespace my_cartographer
{
  namespace map
  {
    namespace optimization
    {

      class SpaCostFunction3D
      {
      public:
        static ceres::CostFunction *CreateAutoDiffCostFunction(
            const PoseGraph::Constraint::Pose &pose)
        {
          return new ceres::AutoDiffCostFunction<
              SpaCostFunction3D, 6 /* residuals */, 4 /* rotation variables */,
              3 /* translation variables */, 4 /* rotation variables */,
              3 /* translation variables */>(new SpaCostFunction3D(pose));
        }

        template <typename T>
        bool operator()(const T *const c_i_rotation, const T *const c_i_translation,
                        const T *const c_j_rotation, const T *const c_j_translation,
                        T *const e) const
        {
          const std::array<T, 6> error = ScaleError(
              ComputeUnscaledError(pose_.zbar_ij, c_i_rotation, c_i_translation,
                                   c_j_rotation, c_j_translation),
              pose_.translation_weight, pose_.rotation_weight);
          std::copy(std::begin(error), std::end(error), e);
          return true;
        }

      private:
        explicit SpaCostFunction3D(const PoseGraph::Constraint::Pose &pose)
            : pose_(pose) {}

        const PoseGraph::Constraint::Pose pose_;
      };

    } // namespace optimization
  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_COST_FUNCTIONS_SPA_COST_FUNCTION_3D_HPP_
