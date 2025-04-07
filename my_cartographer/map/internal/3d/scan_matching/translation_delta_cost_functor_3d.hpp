//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_3D_HPP
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_3D_HPP

#include "my_cartographer/common/math.hpp"

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <cmath>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      // Computes the cost of translating 'translation' to 'target_translation'.
      // Cost increases with the solution's distance from 'target_translation'.
      class TranslationDeltaCostFunctor3D
      {
      public:
        static ceres::CostFunction *CreateAutoDiffCostFunction(
            const double scaling_factor, const Eigen::Vector3d &target_translation)
        {
          return new ceres::AutoDiffCostFunction<TranslationDeltaCostFunctor3D,
                                                 3 /* residuals */,
                                                 3 /* translation variables */>(
              new TranslationDeltaCostFunctor3D(scaling_factor, target_translation));
        }

        template <typename T>
        bool operator()(const T *const translation, T *residual) const
        {
          residual[0] = scaling_factor_ * (translation[0] - x_);
          residual[1] = scaling_factor_ * (translation[1] - y_);
          residual[2] = scaling_factor_ * (translation[2] - z_);
          return true;
        }

      private:
        // Constructs a new TranslationDeltaCostFunctor3D from the given
        // 'target_translation'.
        explicit TranslationDeltaCostFunctor3D(
            const double scaling_factor, const Eigen::Vector3d &target_translation)
            : scaling_factor_(scaling_factor),
              x_(target_translation.x()),
              y_(target_translation.y()),
              z_(target_translation.z()) {}

        TranslationDeltaCostFunctor3D(const TranslationDeltaCostFunctor3D &) = delete;
        TranslationDeltaCostFunctor3D &operator=(
            const TranslationDeltaCostFunctor3D &) = delete;

        const double scaling_factor_;
        const double x_;
        const double y_;
        const double z_;
      };

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_TRANSLATION_DELTA_COST_FUNCTOR_3D_HPP