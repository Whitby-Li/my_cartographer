//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_3D_HPP
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_3D_HPP

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

      // Computes the cost of rotating 'rotation_quaternion' to 'target_rotation'.
      // Cost increases with the solution's distance from 'target_rotation'.
      class RotationDeltaCostFunctor3D
      {
      public:
        static ceres::CostFunction *CreateAutoDiffCostFunction(
            const double scaling_factor, const Eigen::Quaterniond &target_rotation)
        {
          return new ceres::AutoDiffCostFunction<RotationDeltaCostFunctor3D,
                                                 3 /* residuals */,
                                                 4 /* rotation variables */>(
              new RotationDeltaCostFunctor3D(scaling_factor, target_rotation));
        }

        template <typename T>
        bool operator()(const T *const rotation_quaternion, T *residual) const
        {
          std::array<T, 4> delta;
          common::QuaternionProduct(target_rotation_inverse_, rotation_quaternion,
                                    delta.data());
          // Will compute the squared norm of the imaginary component of the delta
          // quaternion which is sin(phi/2)^2.
          residual[0] = scaling_factor_ * delta[1];
          residual[1] = scaling_factor_ * delta[2];
          residual[2] = scaling_factor_ * delta[3];
          return true;
        }

      private:
        // Constructs a new RotationDeltaCostFunctor3D from the given
        // 'target_rotation'.
        explicit RotationDeltaCostFunctor3D(const double scaling_factor,
                                            const Eigen::Quaterniond &target_rotation)
            : scaling_factor_(scaling_factor)
        {
          target_rotation_inverse_[0] = target_rotation.w();
          target_rotation_inverse_[1] = -target_rotation.x();
          target_rotation_inverse_[2] = -target_rotation.y();
          target_rotation_inverse_[3] = -target_rotation.z();
        }

        RotationDeltaCostFunctor3D(const RotationDeltaCostFunctor3D &) = delete;
        RotationDeltaCostFunctor3D &operator=(const RotationDeltaCostFunctor3D &) =
            delete;

        const double scaling_factor_;
        double target_rotation_inverse_[4];
      };

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_ROTATION_DELTA_COST_FUNCTOR_3D_HPP