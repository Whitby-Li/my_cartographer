//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_CERES_POSE_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_CERES_POSE_H

#include "my_cartographer/transform/rigid_transform.h"

#include <ceres/ceres.h>
#include <Eigen/Core>

#include <array>
#include <memory>

namespace my_cartographer
{
  namespace map
  {
    namespace optimization
    {

      class CeresPose
      {
      public:
        CeresPose(
            const transform::Rigid3d &rigid,
            std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
            std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
            ceres::Problem *problem);

        const transform::Rigid3d ToRigid() const;

        double *translation() { return data_->translation.data(); }
        const double *translation() const { return data_->translation.data(); }

        double *rotation() { return data_->rotation.data(); }
        const double *rotation() const { return data_->rotation.data(); }

        struct Data
        {
          std::array<double, 3> translation;
          // Rotation quaternion as (w, x, y, z).
          std::array<double, 4> rotation;
        };

        Data &data() { return *data_; }

      private:
        std::shared_ptr<Data> data_;
      };

      CeresPose::Data FromPose(const transform::Rigid3d &pose);

    } // namespace optimization
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_OPTIMIZATION_CERES_POSE_H