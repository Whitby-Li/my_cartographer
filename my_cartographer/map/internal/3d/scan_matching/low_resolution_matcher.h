//
// Created by whitby on 2025-04-07.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_LOW_RESOLUTION_MATCHER_H
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_LOW_RESOLUTION_MATCHER_H

#include "my_cartographer/map/3d/hybrid_grid.hpp"
#include "my_cartographer/sensor/point_cloud.h"
#include "my_cartographer/transform/rigid_transform.h"

#include <functional>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {

      std::function<float(const transform::Rigid3f &)> CreateLowResolutionMatcher(
          const HybridGrid *low_resolution_grid, const sensor::PointCloud *points);

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer