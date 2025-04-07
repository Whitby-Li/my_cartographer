//
// Created by whitby on 2025-04-06.
//

#include "my_cartographer/map/internal/optimization/ceres_pose.h"

namespace my_cartographer
{
  namespace map
  {
    namespace optimization
    {

      CeresPose::Data FromPose(const transform::Rigid3d &pose)
      {
        return CeresPose::Data{{{pose.translation().x(), pose.translation().y(),
                                 pose.translation().z()}},
                               {{pose.rotation().w(), pose.rotation().x(),
                                 pose.rotation().y(), pose.rotation().z()}}};
      }

      CeresPose::CeresPose(
          const transform::Rigid3d &pose,
          std::unique_ptr<ceres::LocalParameterization> translation_parametrization,
          std::unique_ptr<ceres::LocalParameterization> rotation_parametrization,
          ceres::Problem *problem)
          : data_(std::make_shared<CeresPose::Data>(FromPose(pose)))
      {
        problem->AddParameterBlock(data_->translation.data(), 3,
                                   translation_parametrization.release());
        problem->AddParameterBlock(data_->rotation.data(), 4,
                                   rotation_parametrization.release());
      }

      const transform::Rigid3d CeresPose::ToRigid() const
      {
        return transform::Rigid3d::FromArrays(data_->rotation, data_->translation);
      }

    } // namespace optimization
  } // namespace map
} // namespace my_cartographer