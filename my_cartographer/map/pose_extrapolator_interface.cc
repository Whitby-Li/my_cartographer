//
// Created by whitby on 2025-03-26.
//

#include "my_cartographer/map/pose_extrapolator_interface.h"
#include "my_cartographer/common/internal/ceres_solver_options.h"

namespace my_cartographer
{
  namespace map
  {

    namespace
    {

      proto::ConstantVelocityPoseExtrapolatorOptions CreateConstantVelocityPoseExtrapolatorOptions(
          common::LuaParameterDictionary *const parameter_dictionary)
      {
        proto::ConstantVelocityPoseExtrapolatorOptions options;
        options.set_pose_queue_duration(parameter_dictionary->GetDouble("pose_queue_duration"));
        options.set_imu_gravity_time_constant(parameter_dictionary->GetDouble("imu_gravity_time_constant"));
        return options;
      }

      proto::ImuBasedPoseExtrapolatorOptions CreateImuBasedPoseExtrapolatorOptions(
          common::LuaParameterDictionary *const parameter_dictionary)
      {
        proto::ImuBasedPoseExtrapolatorOptions options;

        options.set_pose_queue_duration(parameter_dictionary->GetDouble("pose_queue_duration"));
        options.set_gravity_constant(parameter_dictionary->GetDouble("gravity_constant"));
        options.set_pose_translation_weight(parameter_dictionary->GetDouble("pose_translation_weight"));
        options.set_pose_rotation_weight(parameter_dictionary->GetDouble("pose_rotation_weight"));
        options.set_imu_acceleration_weight(parameter_dictionary->GetDouble("imu_acceleration_weight"));
        options.set_imu_rotation_weight(parameter_dictionary->GetDouble("imu_rotation_weight"));
        options.set_odometry_rotation_weight(parameter_dictionary->GetDouble("odometry_rotation_weight"));
        options.set_odometry_translation_weight(parameter_dictionary->GetDouble("odometry_translation_weight"));
        *options.mutable_solver_options() = CreateCeresSolverOptionsProto(parameter_dictionary->GetDictionary("solver_options").get());

        return options;
      }

    }

    proto::PoseExtrapolatorOptions CreatePoseExtrapolatorOptions(
        common::LuaParameterDictionary *const parameter_dictionary)
    {
      proto::PoseExtrapolatorOptions options;
      options.set_use_imu_based(parameter_dictionary->GetBool("use_imu_based"));
      *options.mutable_constant_velocity() = CreateConstantVelocityPoseExtrapolatorOptions(
          parameter_dictionary->GetDictionary("constant_velocity").get());
      *options.mutable_imu_based() = CreateImuBasedPoseExtrapolatorOptions(
          parameter_dictionary->GetDictionary("imu_based").get());
    }

    // std::unique_ptr<PoseExtrapolatorInterface> PoseExtrapolatorInterface::CreateWithImuData(
    //     const proto::PoseExtrapolatorOptions &options,
    //     const std::vector<sensor::ImuData> &imu_data,
    //     const std::vector<transform::TimestampedTransform> &initial_poses)
    // {
    //   CHECK(!imu_data.empty());
    //   if (options.use_imu_based())
    //   {
    //     return ImuBasedPoseExtrapolator::InitializeWithImu(options.imu_based(),
    //                                                        imu_data, initial_poses);
    //   }
    //   else
    //   {
    //     return PoseExtrapolator::InitializeWithImu(
    //         common::FromSeconds(options.constant_velocity().pose_queue_duration()),
    //         options.constant_velocity().imu_gravity_time_constant(),
    //         imu_data.back());
    //   }
    // }

  } // namespace map
} // namespace my_cartographer