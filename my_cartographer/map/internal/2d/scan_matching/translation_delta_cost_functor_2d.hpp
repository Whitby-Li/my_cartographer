//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      // Computes the cost of translating 'pose' to 'target_translation'.
      // Cost increases with the solution's distance from 'target_translation'.
      class TranslationDeltaCostFunctor2D
      {
      public:
        static ceres::CostFunction *CreateAutoDiffCostFunction(
            const double scaling_factor, const Eigen::Vector2d &target_translation)
        {
          return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor2D,
                                                 2 /* residuals */,
                                                 3 /* pose variables */>(
              new TranslationDeltaCostFunctor2D(scaling_factor, target_translation));
        }

        template <typename T>
        bool operator()(const T *const pose, T *residual) const
        {
          residual[0] = scaling_factor_ * (pose[0] - x_);
          residual[1] = scaling_factor_ * (pose[1] - y_);
          return true;
        }

      private:
        // Constructs a new TranslationDeltaCostFunctor2D from the given
        // 'target_translation' (x, y).
        explicit TranslationDeltaCostFunctor2D(
            const double scaling_factor, const Eigen::Vector2d &target_translation)
            : scaling_factor_(scaling_factor),
              x_(target_translation.x()),
              y_(target_translation.y()) {}

        TranslationDeltaCostFunctor2D(const TranslationDeltaCostFunctor2D &) = delete;
        TranslationDeltaCostFunctor2D &operator=(
            const TranslationDeltaCostFunctor2D &) = delete;

        const double scaling_factor_;
        const double x_;
        const double y_;
      };

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_2D_H