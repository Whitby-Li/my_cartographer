//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/map/2d/grid_2d.h"
#include "my_cartographer/map/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "my_cartographer/sensor/point_cloud.h"

#include <ceres/ceres.h>
#include <Eigen/Core>

#include <memory>
#include <vector>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      proto::CeresScanMatcherOptions2D CreateCeresScanMatcherOptions2D(
          common::LuaParameterDictionary *parameter_dictionary);

      // Align scans with an existing map using Ceres.
      class CeresScanMatcher2D
      {
      public:
        explicit CeresScanMatcher2D(const proto::CeresScanMatcherOptions2D &options);
        virtual ~CeresScanMatcher2D();

        CeresScanMatcher2D(const CeresScanMatcher2D &) = delete;
        CeresScanMatcher2D &operator=(const CeresScanMatcher2D &) = delete;

        // Aligns 'point_cloud' within the 'grid' given an
        // 'initial_pose_estimate' and returns a 'pose_estimate' and the solver
        // 'summary'.
        void Match(const Eigen::Vector2d &target_translation,
                   const transform::Rigid2d &initial_pose_estimate,
                   const sensor::PointCloud &point_cloud, const Grid2D &grid,
                   transform::Rigid2d *pose_estimate,
                   ceres::Solver::Summary *summary) const;

      private:
        const proto::CeresScanMatcherOptions2D options_;
        ceres::Solver::Options ceres_solver_options_;
      };

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_2D_H