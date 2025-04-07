//
// Created by whitby on 2025-04-06.
//

#include "my_cartographer/map/internal/3d/scan_matching/intensity_cost_function_3d.h"

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matchking
    {

      // This method is defined here instead of the header file as it was observed
      // that defining it in the header file has a negative impact on the runtime
      // performance.
      ceres::CostFunction *IntensityCostFunction3D::CreateAutoDiffCostFunction(
          const double scaling_factor, const float intensity_threshold,
          const sensor::PointCloud &point_cloud,
          const IntensityHybridGrid &hybrid_grid)
      {
        CHECK(!point_cloud.intensities().empty());
        return new ceres::AutoDiffCostFunction<
            IntensityCostFunction3D, ceres::DYNAMIC /* residuals */,
            3 /* translation variables */, 4 /* rotation variables */>(
            new IntensityCostFunction3D(scaling_factor, intensity_threshold,
                                        point_cloud, hybrid_grid),
            point_cloud.size());
      }

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer