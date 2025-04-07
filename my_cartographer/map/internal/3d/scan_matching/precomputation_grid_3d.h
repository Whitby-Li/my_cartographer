//
// Created by whitby on 2025-04-07.
//

#ifndef MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_PRECOMPUTATION_GRID_3D_H_
#define MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_PRECOMPUTATION_GRID_3D_H_

#include "my_cartographer/map/3d/hybrid_grid.hpp"

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {
      class PrecomputationGrid3D : public HybridGridBase<uint8>
      {
      public:
        explicit PrecomputationGrid3D(const float resolution)
            : HybridGridBase<uint8>(resolution) {}

        // Maps values from [0, 255] to [kMinProbability, kMaxProbability].
        static float ToProbability(float value)
        {
          return kMinProbability +
                 value * ((kMaxProbability - kMinProbability) / 255.f);
        }
      };

      // Converts a HybridGrid to a PrecomputationGrid3D representing the same data,
      // but only using 8 bit instead of 2 x 16 bit.
      PrecomputationGrid3D ConvertToPrecomputationGrid(const HybridGrid &hybrid_grid);

      // Returns a grid of the same resolution containing the maximum value of
      // original voxels in 'grid'. This maximum is over the 8 voxels that have
      // any combination of index components optionally increased by 'shift'.
      // If 'shift' is 2 ** (depth - 1), where depth 0 is the original grid, and this
      // is using the precomputed grid of one depth before, this results in
      // precomputation grids analogous to the 2D case.
      PrecomputationGrid3D PrecomputeGrid(const PrecomputationGrid3D &grid,
                                          bool half_resolution,
                                          const Eigen::Array3i &shift);
    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer

#endif // MY_CARTOGRAPHER_MAP_INTERNAL_3D_SCAN_MATCHING_PRECOMPUTATION_GRID_3D_H_