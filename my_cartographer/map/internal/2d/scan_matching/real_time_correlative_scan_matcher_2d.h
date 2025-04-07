//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H

#include "my_cartographer/map/2d/grid_2d.h"
#include "my_cartographer/map/internal/2d/scan_matching/correlative_scan_matcher_2d.h"
#include "my_cartographer/map/proto/scan_matching/real_time_correlative_scan_matcher_options.pb.h"

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      // An implementation of "Real-Time Correlative Scan Matching" by Olson.
      class RealTimeCorrelativeScanMatcher2D
      {
      public:
        explicit RealTimeCorrelativeScanMatcher2D(
            const proto::RealTimeCorrelativeScanMatcherOptions &options);

        RealTimeCorrelativeScanMatcher2D(const RealTimeCorrelativeScanMatcher2D &) =
            delete;
        RealTimeCorrelativeScanMatcher2D &operator=(
            const RealTimeCorrelativeScanMatcher2D &) = delete;

        // Aligns 'point_cloud' within the 'grid' given an
        // 'initial_pose_estimate' then updates 'pose_estimate' with the result and
        // returns the score.
        double Match(const transform::Rigid2d &initial_pose_estimate,
                     const sensor::PointCloud &point_cloud, const Grid2D &grid,
                     transform::Rigid2d *pose_estimate) const;

        // Computes the score for each Candidate2D in a collection. The cost is
        // computed as the sum of probabilities or normalized TSD values, different
        // from the Ceres CostFunctions: http://ceres-solver.org/modeling.html
        //
        // Visible for testing.
        void ScoreCandidates(const Grid2D &grid,
                             const std::vector<DiscreteScan2D> &discrete_scans,
                             const SearchParameters &search_parameters,
                             std::vector<Candidate2D> *candidates) const;

      private:
        std::vector<Candidate2D> GenerateExhaustiveSearchCandidates(
            const SearchParameters &search_parameters) const;

        const proto::RealTimeCorrelativeScanMatcherOptions options_;
      };

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_2D_H