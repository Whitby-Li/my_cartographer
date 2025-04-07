//
// Created by whitby on 2025-04-07.
//

#include "my_cartographer/map/internal/3d/scan_matching/precomputation_grid_3d.h"
#include "my_cartographer/map/probability_values.h"
#include "my_cartographer/common/math.hpp"

#include <Eigen/Core>
#include <glog/logging.h>

#include <algorithm>

namespace my_cartographer
{
  namespace map
  {
    namespace scan_matching
    {
      namespace
      {
        // C++11 defines that integer division rounds towards zero. For index math, we
        // actually need it to round towards negative infinity. Luckily bit shifts have
        // that property.
        inline int DivideByTwoRoundingTowardsNegativeInfinity(const int value)
        {
          return value >> 1;
        }

        // Computes the half resolution index corresponding to the full resolution
        // 'cell_index'.
        Eigen::Array3i CellIndexAtHalfResolution(const Eigen::Array3i &cell_index)
        {
          return Eigen::Array3i(
              DivideByTwoRoundingTowardsNegativeInfinity(cell_index[0]),
              DivideByTwoRoundingTowardsNegativeInfinity(cell_index[1]),
              DivideByTwoRoundingTowardsNegativeInfinity(cell_index[2]));
        }

      } // namespace

      PrecomputationGrid3D ConvertToPrecomputationGrid(
          const HybridGrid &hybrid_grid)
      {
        PrecomputationGrid3D result(hybrid_grid.resolution());
        for (auto it = HybridGrid::Iterator(hybrid_grid); !it.Done(); it.Next())
        {
          const int cell_value = common::RoundToInt(
              (ValueToProbability(it.GetValue()) - kMinProbability) *
              (255.f / (kMaxProbability - kMinProbability)));
          CHECK_GE(cell_value, 0);
          CHECK_LE(cell_value, 255);
          *result.mutable_value(it.GetCellIndex()) = cell_value;
        }
        return result;
      }

      PrecomputationGrid3D PrecomputeGrid(const PrecomputationGrid3D &grid,
                                          const bool half_resolution,
                                          const Eigen::Array3i &shift)
      {
        PrecomputationGrid3D result(grid.resolution());
        for (auto it = PrecomputationGrid3D::Iterator(grid); !it.Done(); it.Next())
        {
          for (int i = 0; i != 8; ++i)
          {
            // We use this value to update 8 values in the resulting grid, at
            // position (x - {0, 'shift'}, y - {0, 'shift'}, z - {0, 'shift'}).
            // If 'shift' is 2 ** (depth - 1), where depth 0 is the original grid,
            // this results in precomputation grids analogous to the 2D case.
            const Eigen::Array3i cell_index =
                it.GetCellIndex() - shift * PrecomputationGrid3D::GetOctant(i);
            auto *const cell_value = result.mutable_value(
                half_resolution ? CellIndexAtHalfResolution(cell_index) : cell_index);
            *cell_value = std::max(it.GetValue(), *cell_value);
          }
        }
        return result;
      }

    } // namespace scan_matching
  } // namespace map
} // namespace my_cartographer