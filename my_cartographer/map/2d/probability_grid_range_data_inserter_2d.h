//
// Created by whitby on 2025-04-05.
//

#ifndef MY_CARTOGRAPHER_MAP_2D_PROBABILITY_GRID_RANGE_DATA_INSERTER_2D_H
#define MY_CARTOGRAPHER_MAP_2D_PROBABILITY_GRID_RANGE_DATA_INSERTER_2D_H

#include "my_cartographer/common/lua_parameter_dictionary.h"
#include "my_cartographer/common/port.hpp"
#include "my_cartographer/map/2d/probability_grid.h"
#include "my_cartographer/map/2d/xy_index.hpp"
#include "my_cartographer/map/proto/probability_grid_range_data_inserter_options_2d.pb.h"
#include "my_cartographer/map/range_data_inserter_interface.h"
#include "my_cartographer/sensor/point_cloud.h"
#include "my_cartographer/sensor/range_data.h"

#include <utility>
#include <vector>

namespace my_cartographer
{
  namespace map
  {

    proto::ProbabilityGridRangeDataInserterOptions2D
    CreateProbabilityGridRangeDataInserterOptions2D(
        common::LuaParameterDictionary *parameter_dictionary);

    class ProbabilityGridRangeDataInserter2D : public RangeDataInserterInterface
    {
    public:
      explicit ProbabilityGridRangeDataInserter2D(
          const proto::ProbabilityGridRangeDataInserterOptions2D &options);

      ProbabilityGridRangeDataInserter2D(
          const ProbabilityGridRangeDataInserter2D &) = delete;
      ProbabilityGridRangeDataInserter2D &operator=(
          const ProbabilityGridRangeDataInserter2D &) = delete;

      // Inserts 'range_data' into 'probability_grid'.
      virtual void Insert(const sensor::RangeData &range_data,
                          GridInterface *grid) const override;

    private:
      const proto::ProbabilityGridRangeDataInserterOptions2D options_;
      const std::vector<uint16> hit_table_;
      const std::vector<uint16> miss_table_;
    };

  } // namespace map
} // namespace my_cartographer
#endif // MY_CARTOGRAPHER_MAP_2D_PROBABILITY_GRID_RANGE_DATA_INSERTER_2D_H