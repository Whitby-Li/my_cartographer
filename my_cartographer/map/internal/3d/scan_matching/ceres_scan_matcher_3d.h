//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/map/3d/hybrid_grid.hpp"
#include "my_cartographer/map/proto/scan_matching/ceres_scan_matcher_options_3d.pb.h"
#include "my_cartographer/sensor/point_cloud.h"
#include "my_cartographer/transform/rigid_transform.h"

#include <Eigen/Core>

#include <utility>
#include <vector>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      proto::CeresScanMatcherOptions3D CreateCeresScanMatcherOptions3D(
          common::LuaParameterDictionary *parameter_dictionary);

      struct PointCloudAndHybridGridsPointers
      {
        const sensor::PointCloud *point_cloud;
        const HybridGrid *hybrid_grid;
        const IntensityHybridGrid *intensity_hybrid_grid; // optional
      };

      // This scan matcher uses Ceres to align scans with an existing map.
      class CeresScanMatcher3D
      {
      public:
        explicit CeresScanMatcher3D(const proto::CeresScanMatcherOptions3D &options);

        CeresScanMatcher3D(const CeresScanMatcher3D &) = delete;
        CeresScanMatcher3D &operator=(const CeresScanMatcher3D &) = delete;

        // Aligns 'point_clouds' within the 'hybrid_grids' given an
        // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
        // 'summary'.
        void Match(const Eigen::Vector3d &target_translation,
                   const transform::Rigid3d &initial_pose_estimate,
                   const std::vector<PointCloudAndHybridGridsPointers> &
                       point_clouds_and_hybrid_grids,
                   transform::Rigid3d *pose_estimate,
                   ceres::Solver::Summary *summary) const;

      private:
        const proto::CeresScanMatcherOptions3D options_;
        ceres::Solver::Options ceres_solver_options_;
      };

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_CERES_SCAN_MATCHER_3D_H