//
// Created by whitby on 2025-04-06.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H

#include "my_cartographer/map/2d/grid_2d.h"
#include "my_cartographer/sensor/point_cloud.h"

#include <ceres/ceres.h>

namespace cartographer
{
  namespace map
  {
    namespace scan_matching
    {
      // Creates a cost function for matching the 'point_cloud' to the 'grid' with
      // a 'pose'. The cost increases with poorer correspondence of the grid and the
      // point observation (e.g. points falling into less occupied space).
      ceres::CostFunction *CreateOccupiedSpaceCostFunction2D(
          const double scaling_factor, const sensor::PointCloud &point_cloud,
          const Grid2D &grid);

    } // namespace scan_matching
  } // namespace map
} // namespace cartographer
#endif // MY_CARTOGRAPHER_MAP_INTERNAL_2D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTION_2D_H