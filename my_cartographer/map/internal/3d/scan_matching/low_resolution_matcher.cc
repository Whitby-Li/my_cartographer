//
// Created by whitby on 2025-04-07.
//

#include "my_cartographer/map/internal/3d/scan_matching/low_resolution_matcher.h"

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      std::function<float(const transform::Rigid3f &)> CreateLowResolutionMatcher(
          const HybridGrid *low_resolution_grid, const sensor::PointCloud *points)
      {
        return [=](const transform::Rigid3f &pose)
        {
          float score = 0.f;
          for (const sensor::RangefinderPoint &point :
               sensor::TransformPointCloud(*points, pose))
          {
            // TODO(zhengj, whess): Interpolate the Grid to get better score.
            score += low_resolution_grid->GetProbability(
                low_resolution_grid->GetCellIndex(point.position));
          }
          return score / points->size();
        };
      }
      
    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer