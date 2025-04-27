//
// Created by whitby on 2025-04-12.
//

#include "my_cartographer/map/internal/2d/local_trajectory_builder_options_2d.h"
#include "my_cartographer/map/2d/submap_2d.h"
#include "my_cartographer/map/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "my_cartographer/map/internal/motion_filter.h"
#include "my_cartographer/map/internal/scan_matching/real_time_correlative_scan_matcher.h"
#include "my_cartographer/map/pose_extrapolator_interface.h"
#include "my_cartographer/sensor/internal/voxel_filter.h"

namespace my_cartographer
{
  namespace map
  {

    proto::LocalTrajectoryBuilderOptions2D CreateLocalTrajectoryBuilderOptions2D(
        common::LuaParameterDictionary *const parameter_dictionary)
    {
      proto::LocalTrajectoryBuilderOptions2D options;
      options.set_min_range(parameter_dictionary->GetDouble("min_range"));
      options.set_max_range(parameter_dictionary->GetDouble("max_range"));
      options.set_min_z(parameter_dictionary->GetDouble("min_z"));
      options.set_max_z(parameter_dictionary->GetDouble("max_z"));
      options.set_missing_data_ray_length(
          parameter_dictionary->GetDouble("missing_data_ray_length"));
      options.set_num_accumulated_range_data(
          parameter_dictionary->GetInt("num_accumulated_range_data"));
      options.set_voxel_filter_size(
          parameter_dictionary->GetDouble("voxel_filter_size"));
      options.set_use_online_correlative_scan_matching(
          parameter_dictionary->GetBool("use_online_correlative_scan_matching"));
      *options.mutable_adaptive_voxel_filter_options() =
          sensor::CreateAdaptiveVoxelFilterOptions(
              parameter_dictionary->GetDictionary("adaptive_voxel_filter").get());
      *options.mutable_loop_closure_adaptive_voxel_filter_options() =
          sensor::CreateAdaptiveVoxelFilterOptions(
              parameter_dictionary
                  ->GetDictionary("loop_closure_adaptive_voxel_filter")
                  .get());
      *options.mutable_real_time_correlative_scan_matcher_options() =
          scan_matching::CreateRealTimeCorrelativeScanMatcherOptions(
              parameter_dictionary
                  ->GetDictionary("real_time_correlative_scan_matcher")
                  .get());
      *options.mutable_ceres_scan_matcher_options() =
          scan_matching::CreateCeresScanMatcherOptions2D(
              parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
      *options.mutable_motion_filter_options() = CreateMotionFilterOptions(
          parameter_dictionary->GetDictionary("motion_filter").get());
      *options.mutable_pose_extrapolator_options() = CreatePoseExtrapolatorOptions(
          parameter_dictionary->GetDictionary("pose_extrapolator").get());
      options.set_imu_gravity_time_constant(
          parameter_dictionary->GetDouble("imu_gravity_time_constant"));
      *options.mutable_submaps_options() = CreateSubmapsOptions2D(
          parameter_dictionary->GetDictionary("submaps").get());
      options.set_use_imu_data(parameter_dictionary->GetBool("use_imu_data"));
      return options;
    }

  } // namespace map
} // namespace my_cartographer