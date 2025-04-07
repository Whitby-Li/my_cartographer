//
// Created by whitby on 2025-04-07.
//

#ifndef MY_CARTOGRAPHER_MAP_3D_RANGE_DATA_INSERTER_3D_H
#define MY_CARTOGRAPHER_MAP_3D_RANGE_DATA_INSERTER_3D_H

#include "my_cartographer/map/3d/hybrid_grid.hpp"
#include "my_cartographer/map/proto/range_data_inserter_options_3d.pb.h"
#include "my_cartographer/sensor/point_cloud.h"
#include "my_cartographer/sensor/range_data.h"

namespace my_cartographer
{
  namespace map
  {

    proto::RangeDataInserterOptions3D CreateRangeDataInserterOptions3D(
        common::LuaParameterDictionary *parameter_dictionary);

    class RangeDataInserter3D
    {
    public:
      explicit RangeDataInserter3D(
          const proto::RangeDataInserterOptions3D &options);

      RangeDataInserter3D(const RangeDataInserter3D &) = delete;
      RangeDataInserter3D &operator=(const RangeDataInserter3D &) = delete;

      // Inserts 'range_data' into 'hybrid_grid' and optionally into
      // 'intensity_hybrid_grid'.
      void Insert(const sensor::RangeData &range_data, HybridGrid *hybrid_grid,
                  IntensityHybridGrid *intensity_hybrid_grid) const;

    private:
      const proto::RangeDataInserterOptions3D options_;
      const std::vector<uint16> hit_table_;
      const std::vector<uint16> miss_table_;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_3D_RANGE_DATA_INSERTER_3D_H