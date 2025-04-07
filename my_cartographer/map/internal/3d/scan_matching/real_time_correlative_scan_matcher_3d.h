//
// Created by whitby on 2025-04-07.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_3D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_3D_H

#include "my_cartographer/map/3d/hybrid_grid.hpp"
#include "my_cartographer/map/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"
#include "my_cartographer/sensor/point_cloud.h"

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      // A voxel accurate scan matcher, exhaustively evaluating the scan matching
      // search space.
      class RealTimeCorrelativeScanMatcher3D
      {
      public:
        explicit RealTimeCorrelativeScanMatcher3D(
            const scan_matching::proto::RealTimeCorrelativeScanMatcherOptions &
                options);

        RealTimeCorrelativeScanMatcher3D(const RealTimeCorrelativeScanMatcher3D &) =
            delete;
        RealTimeCorrelativeScanMatcher3D &operator=(
            const RealTimeCorrelativeScanMatcher3D &) = delete;

        // Aligns 'point_cloud' within the 'hybrid_grid' given an
        // 'initial_pose_estimate' then updates 'pose_estimate' with the result and
        // returns the score.
        float Match(const transform::Rigid3d &initial_pose_estimate,
                    const sensor::PointCloud &point_cloud,
                    const HybridGrid &hybrid_grid,
                    transform::Rigid3d *pose_estimate) const;

      private:
        std::vector<transform::Rigid3f> GenerateExhaustiveSearchTransforms(
            float resolution, const sensor::PointCloud &point_cloud) const;
        float ScoreCandidate(const HybridGrid &hybrid_grid,
                             const sensor::PointCloud &transformed_point_cloud,
                             const transform::Rigid3f &transform) const;

        const proto::RealTimeCorrelativeScanMatcherOptions options_;
      };

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_3D_H