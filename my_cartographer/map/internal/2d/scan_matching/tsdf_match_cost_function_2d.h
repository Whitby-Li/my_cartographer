//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_TSDF_MATCH_COST_FUNCTION_2D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_TSDF_MATCH_COST_FUNCTION_2D_H

#include "my_cartographer/map/internal/2d/tsdf_2d.h"
#include "my_cartographer/sensor/point_cloud.h"

#include <ceres/ceres.h>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matchking
    {

      // Creates a cost function for matching the 'point_cloud' in the 'grid' at
      // a 'pose'. The cost increases with the signed distance of the matched point
      // location in the 'grid'.
      ceres::CostFunction *CreateTSDFMatchCostFunction2D(
          const double scaling_factor, const sensor::PointCloud &point_cloud,
          const TSDF2D &grid);

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_TSDF_MATCH_COST_FUNCTION_2D_H