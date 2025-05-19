//
// Created by whitby on 2025-05-05.
//

#include "my_cartographer/map/internal/optimization/optimization_problem_options.h"

#include "my_cartographer/common/internal/ceres_solver_options.h"

namespace my_cartographer
{
  namespace map
  {
    namespace optimization
    {

      proto::OptimizationProblemOptions CreateOptimizationProblemOptions(
          common::LuaParameterDictionary *const parameter_dictionary)
      {
        proto::OptimizationProblemOptions options;
        options.set_huber_scale(parameter_dictionary->GetDouble("huber_scale"));
        options.set_acceleration_weight(
            parameter_dictionary->GetDouble("acceleration_weight"));
        options.set_rotation_weight(
            parameter_dictionary->GetDouble("rotation_weight"));
        options.set_odometry_translation_weight(
            parameter_dictionary->GetDouble("odometry_translation_weight"));
        options.set_odometry_rotation_weight(
            parameter_dictionary->GetDouble("odometry_rotation_weight"));
        options.set_local_slam_pose_translation_weight(
            parameter_dictionary->GetDouble("local_slam_pose_translation_weight"));
        options.set_local_slam_pose_rotation_weight(
            parameter_dictionary->GetDouble("local_slam_pose_rotation_weight"));
        options.set_fixed_frame_pose_translation_weight(
            parameter_dictionary->GetDouble("fixed_frame_pose_translation_weight"));
        options.set_fixed_frame_pose_rotation_weight(
            parameter_dictionary->GetDouble("fixed_frame_pose_rotation_weight"));
        options.set_fixed_frame_pose_use_tolerant_loss(
            parameter_dictionary->GetBool("fixed_frame_pose_use_tolerant_loss"));
        options.set_fixed_frame_pose_tolerant_loss_param_a(
            parameter_dictionary->GetDouble("fixed_frame_pose_tolerant_loss_param_a"));
        options.set_fixed_frame_pose_tolerant_loss_param_b(
            parameter_dictionary->GetDouble("fixed_frame_pose_tolerant_loss_param_b"));
        options.set_log_solver_summary(
            parameter_dictionary->GetBool("log_solver_summary"));
        options.set_use_online_imu_extrinsics_in_3d(
            parameter_dictionary->GetBool("use_online_imu_extrinsics_in_3d"));
        options.set_fix_z_in_3d(parameter_dictionary->GetBool("fix_z_in_3d"));
        *options.mutable_ceres_solver_options() =
            common::CreateCeresSolverOptionsProto(
                parameter_dictionary->GetDictionary("ceres_solver_options").get());
        return options;
      }

    } // namespace optimization
  } // namespace map
} // namespace my_cartographer