//
// Created by whitby on 2025-04-06.
//

#include "my_cartographer/map/internal/constraints/constraint_builder.h"
#include "my_cartographer/map/internal/2d/scan_matching/ceres_scan_matcher_2d.h"
#include "my_cartographer/map/internal/2d/scan_matching/fast_correlative_scan_matcher_2d.h"
#include "my_cartographer/map/internal/3d/scan_matching/ceres_scan_matcher_3d.h"
#include "my_cartographer/map/internal/3d/scan_matching/fast_correlative_scan_matcher_3d.h"
#include "my_cartographer/sensor/internal/voxel_filter.h"

namespace my_cartographer
{
  namespace map
  {
    namespace constraints
    {

      proto::ConstraintBuilderOptions CreateConstraintBuilderOptions(
          common::LuaParameterDictionary *const parameter_dictionary)
      {
        proto::ConstraintBuilderOptions options;
        options.set_sampling_ratio(parameter_dictionary->GetDouble("sampling_ratio"));
        options.set_max_constraint_distance(
            parameter_dictionary->GetDouble("max_constraint_distance"));
        options.set_min_score(parameter_dictionary->GetDouble("min_score"));
        options.set_global_localization_min_score(
            parameter_dictionary->GetDouble("global_localization_min_score"));
        options.set_loop_closure_translation_weight(
            parameter_dictionary->GetDouble("loop_closure_translation_weight"));
        options.set_loop_closure_rotation_weight(
            parameter_dictionary->GetDouble("loop_closure_rotation_weight"));
        options.set_log_matches(parameter_dictionary->GetBool("log_matches"));
        *options.mutable_fast_correlative_scan_matcher_options() =
            scan_matching::CreateFastCorrelativeScanMatcherOptions2D(
                parameter_dictionary->GetDictionary("fast_correlative_scan_matcher")
                    .get());
        *options.mutable_ceres_scan_matcher_options() =
            scan_matching::CreateCeresScanMatcherOptions2D(
                parameter_dictionary->GetDictionary("ceres_scan_matcher").get());
        *options.mutable_fast_correlative_scan_matcher_options_3d() =
            scan_matching::CreateFastCorrelativeScanMatcherOptions3D(
                parameter_dictionary
                    ->GetDictionary("fast_correlative_scan_matcher_3d")
                    .get());
        *options.mutable_ceres_scan_matcher_options_3d() =
            scan_matching::CreateCeresScanMatcherOptions3D(
                parameter_dictionary->GetDictionary("ceres_scan_matcher_3d").get());
        return options;
      }

    } // namespace constraints
  } // namespace map
} // namespace my_cartographer