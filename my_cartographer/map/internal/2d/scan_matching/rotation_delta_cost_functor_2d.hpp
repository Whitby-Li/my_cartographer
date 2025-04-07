//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_2D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_2D_H

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      // Computes the cost of rotating 'pose' to 'target_angle'. Cost increases with
      // the solution's distance from 'target_angle'.
      class RotationDeltaCostFunctor2D
      {
      public:
        static ceres::CostFunction *CreateAutoDiffCostFunction(
            const double scaling_factor, const double target_angle)
        {
          return new ceres::AutoDiffCostFunction<
              RotationDeltaCostFunctor2D, 1 /* residuals */, 3 /* pose variables */>(
              new RotationDeltaCostFunctor2D(scaling_factor, target_angle));
        }

        template <typename T>
        bool operator()(const T *const pose, T *residual) const
        {
          residual[0] = scaling_factor_ * (pose[2] - angle_);
          return true;
        }

      private:
        explicit RotationDeltaCostFunctor2D(const double scaling_factor,
                                            const double target_angle)
            : scaling_factor_(scaling_factor), angle_(target_angle) {}

        RotationDeltaCostFunctor2D(const RotationDeltaCostFunctor2D &) = delete;
        RotationDeltaCostFunctor2D &operator=(const RotationDeltaCostFunctor2D &) =
            delete;

        const double scaling_factor_;
        const double angle_;
      };

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_2D_H